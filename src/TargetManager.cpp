//
// Created by jbs on 21. 6. 7..
//

#include <chasing_utils/TargetManager.h>

namespace chasing_utils{
    /**
     *
     * @param evalTime
     * @return
     * @bug if evalTime is beyond the time horizon, just will evalute the final point
     */
    Point PredictionOutput::eval(ros::Time evalTime) const {
        double tEval = (evalTime - refTime).toSec();
        return predictionTraj.eval(tEval);
    }

    pcl::PointCloud<pcl::PointXYZI> Target::visualizeObservation(string worldFrameId) const {
        pcl::PointCloud<pcl::PointXYZI> pclPoints;
        if (observationRawQueue.size()) {
            pclPoints.header.frame_id = worldFrameId;
            pclPoints.header.stamp = pcl_conversions::toPCL(observationRawQueue.back().first);
            pclPoints.clear();
            int n = 0;
            for (auto pnt: observationRawQueue) {
                pcl::PointXYZI pclPnt;
                pclPnt.x = pnt.second.x;
                pclPnt.y = pnt.second.y;
                pclPnt.z = pnt.second.z;
                pclPnt.intensity = (pnt.first - observationRawQueue.back().first).toSec();
                pclPoints.push_back(pclPnt);
            }
        }
        return pclPoints;
    }

    nav_msgs::Path  Target::visualizePrediction(string worldFrameId) const {
        nav_msgs::Path prediction;
        if (isPredicted) {
            prediction = curPrediction.getTraj().toNavPath(worldFrameId);
            prediction.header.stamp = ros::Time::now();
        }
        return prediction;
    }

    // assume feasiblePredictionMarker is already updated
    visualization_msgs::MarkerArray Target::visualizeFeasibleSet() const {
        visualization_msgs::MarkerArray markerArray = feasiblePredictionMarker;
        ros::Time t = ros::Time::now();
        for(auto& marker: markerArray.markers)
            marker.header.stamp = t;
        return markerArray;
    }

    /**
     * Parse paramter from yaml file. Assuming the param file is loaded with ns = "target_manager/*"
     * @param nhTarget
     */
    void TargetManager::Param::parseFrom(const ros::NodeHandle &nhTarget) {
        nhTarget.getParam("world_frame_id",worldFrameId);

        nhTarget.getParam("observation/interval",observationInterval);
        nhTarget.getParam("observation/queue_size",observationQueueSize);
        nhTarget.getParam("observation/queue_min",nMinObservationForPrediction);

        nhTarget.getParam("prediction/collision_margin",predictionRiskTol);
        nhTarget.getParam("prediction/weight_shape_change",weightOnShapeChange);
        nhTarget.getParam("prediction/tol/accum_error",accumErrorTol);
        nhTarget.getParam("prediction/tol/age",predictionAgeTol);
        nhTarget.getParam("prediction/tol/risk",predictionRiskTol);
    }


    /**
     * Return various parameters to determine feasibility of trajs. in library
     * @return
     */
    lib::FeasibilityParam TargetManager::getFeasibilityTestingParamBase(){
        lib::FeasibilityParam testingParam;
        testingParam.edtServerPtr = edtServerPtr;
        testingParam.margin = param.predictionCollisionMargin;
        return testingParam;
    }

    TargetManager::TargetManager(octomap_server::EdtOctomapServer* edtOctomapServer) :
    edtServerPtr(edtOctomapServer),nh("~")  {

        // parse parameter server
        initROS();

        if (edtOctomapServer != nullptr)
            state.isEdtExist = true;

        for (int n = 0; n < param.nTarget ; n++)
            state.predictionAccumErrors.push_back(0.0);

        asyncSpinnerPtr->start(); // for target observation callback
    }

    void TargetManager::initROS() {

        nh.getParam("target_frame_set",param.targetObservationFrameId);
        param.nTarget = param.targetObservationFrameId.size();

        if (param.nTarget == 0)
            ROS_ERROR("no target frame id given to be tracked as target. Exiting program");

        observationCallbackQueue.clear();
        nhTimer.setCallbackQueue(&observationCallbackQueue);
        observationTimerCaller  = nhTimer.createTimer(ros::Duration(param.observationInterval),
                                                      &TargetManager::observationTimerCallback, this);
        asyncSpinnerPtr = new ros::AsyncSpinner(1, &observationCallbackQueue);

        mainTimerCaller = nh.createTimer(ros::Duration(0.05),&TargetManager::mainTimerCallback,this);

        tfListener = new tf::TransformListener;
        tfBroadcaster = new tf::TransformBroadcaster;

        // library parameter parsing
        ros::NodeHandle nhLib (nh,"target_library");
        lib::LibraryParam libParam(nhLib);

        // target manager parameter parsing
        param.horizon = libParam.horizon;
        ros::NodeHandle nhManager(nh,"target_manager");
        param.parseFrom(nhManager);

        for (int n =0; n < param.nTarget ; n++){
            Target target;
            string topicPrefix = "target_" + to_string(n);
            target.myIndex = n;
            target.libraryPtr = new lib::LibraryOnline(libParam,0); // 0 does not matther
            targets.push_back(target);
            pubSet.observationRawQueueSet.push_back(
                    nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(topicPrefix + "/observation_keyframe",1));
            pubSet.predictionSet.push_back(
                    nh.advertise<nav_msgs::Path>(topicPrefix + "/prediction",1));
            pubSet.curPredictionPointSet.push_back(
                    nh.advertise<geometry_msgs::PointStamped>(topicPrefix + "/cur_prediction_pnt",1));
            pubSet.curFeasibleCandidateSet.push_back(
                    nh.advertise<visualization_msgs::MarkerArray>(topicPrefix + "/feasible_candid",5));
            pubSet.curPredictionErrorSet.push_back(
                    nh.advertise<std_msgs::Float32>(topicPrefix + "/prediction_error",1));
            pubSet.curTraverseGridSet.push_back(
                    nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(topicPrefix + "/traverse_grid",1));
        }
    }

    /**
     * Callback in a separate thread (async spinner) with defined interval (intended to run not too fast).
     * @param event
     */
    void TargetManager::observationTimerCallback(const ros::TimerEvent &event) {

        /**
         * Collect raw observation in the form of tf
         */
        ros::Time curTime = ros::Time::now();
        for (int n = 0 ; n < param.nTarget ; n++) {
            tf::StampedTransform stampedTransform;
            stampedTransform.setIdentity();
            try {
                // setting ros::Time::now for looking up cannot retrieve enough information in the buffer for this case
                tfListener->lookupTransform(param.worldFrameId, param.targetObservationFrameId[n],
                                            ros::Time(0), stampedTransform);
                updateObservation(n,(stampedTransform));
            } catch (tf::TransformException &ex) {
                ROS_WARN("TargetManager: %s not found. We use position of the tf for prediction.",
                         param.targetObservationFrameId[n].c_str());
//                ROS_INFO_STREAM(ex.what());
            }
            float elapse =  (curTime - targets[n].tLastUpdate).toSec();
            if ( elapse > param.observationInterval * 1.2)
                ROS_WARN("Target tf (%s) is not updated for %f s",
                         param.targetObservationFrameId[n].c_str(),elapse);
        }
    }


    /**
     * Timer callback of global callback queue. Run faster than observation callback.
     * @param event
     */
    void TargetManager::mainTimerCallback(const ros::TimerEvent& event){

        for (int n = 0 ; n < param.nTarget ; n++){
            // Publish group
            if (targets[n].isPoseUpdate)
                pubSet.observationRawQueueSet[n].publish(targets[n].visualizeObservation(param.worldFrameId));
            if (targets[n].isPredicted) {
                // library develop frame
                Pose libRefPose = targets[n].libraryPtr->getRefPose();
                auto tfStamped = libRefPose.toTf(param.worldFrameId,targets[n].getLibRefFrameName(),ros::Time::now());
                tfBroadcaster->sendTransform(tfStamped);

                // feasible candidates
                pubSet.curFeasibleCandidateSet[n].publish(targets[n].visualizeFeasibleSet()); // w.r.t libRefFrame
                pubSet.curTraverseGridSet[n].publish(targets[n].libraryPtr->getMarkerCells(param.worldFrameId));

                // final prediction trajectory
                pubSet.predictionSet[n].publish(targets[n].visualizePrediction(param.worldFrameId));

                // current evaluation on the prediction trajectory
                Point curPredPoint = targets[n].curPrediction.eval(ros::Time::now());
                geometry_msgs::PointStamped pntStamped;
                pntStamped.header.frame_id = param.worldFrameId;
                pntStamped.header.stamp = ros::Time::now();
                pntStamped.point = curPredPoint.toGeometry();
                pubSet.curPredictionPointSet[n].publish(pntStamped);

                // Monitor prediction error
                tf::StampedTransform stampedTransform;
                stampedTransform.setIdentity();
                try {
                    // setting ros::Time::now for looking up cannot retrieve enough information in the buffer for this case
                    tfListener->lookupTransform(param.worldFrameId, param.targetObservationFrameId[n],
                                                ros::Time(0), stampedTransform);
                } catch (tf::TransformException &ex) {
                    ROS_WARN("TargetManager: %s not found. We use position of the tf for prediction.",
                             param.targetObservationFrameId[n].c_str());
                }
                Point curPoint(stampedTransform.getOrigin().x(),
                               stampedTransform.getOrigin().y(),
                               stampedTransform.getOrigin().z());
                float error = curPredPoint.distTo(curPoint);
                state.predictionAccumErrors[n] += error;
                std_msgs::Float32 errorMsg; errorMsg.data = error;
                pubSet.curPredictionErrorSet[n].publish(errorMsg);
            }
        }
    }

    void TargetManager::updateObservation(int targetIdx,
                                          const tf::StampedTransform&  stampedTransform) {

        ros::Time t = stampedTransform.stamp_;
        Pose curPose =  Pose(stampedTransform);
        Point location = curPose.getTranslation();
        Observation observation = make_pair(t, location);

        if (mutex_.try_lock()) {
            ROS_DEBUG("TargetManager: update observation locked");
            auto &observationQueue = targets[targetIdx].observationRawQueue;
            targets[targetIdx].isPoseUpdate = true;
            targets[targetIdx].tLastUpdate = t;
            targets[targetIdx].curPose = curPose;
            observationQueue.push_back(observation);
            if (observationQueue.size() > param.observationQueueSize)
                observationQueue.pop_front();
            mutex_.unlock();
            ROS_DEBUG("TargetManager: update observation unlocked");
        }else
            ROS_WARN_STREAM(myName + ": observation update locked due to prediction");
    }

    bool TargetManager::predict(vector<PredictionOutput> &predictionOutputSet) {
        predictionOutputSet.clear();

        // checking whether all targets received observation from tf lookup
        bool isAllPredictable = true;
        for (int n = 0; n < param.nTarget; n++)
            isAllPredictable = isAllPredictable and
                    (targets[n].observationRawQueue.size() > param.nMinObservationForPrediction);
        if (not isAllPredictable) {
            ROS_ERROR_STREAM(myName + ": prediction not available. Ensure all targets received pose from tf");
            return false;
        }

        // assign cost and feasibility for all trajectories in library
        mutex_.lock();
        ros::Time predictionStartTime = ros::Time::now();
        Timer timer;
        for (int n = 0; n < param.nTarget; n++){
            // set lib ref frame
            Pose curPose = targets[n].curPose;
            targets[n].libraryPtr->updateLibRefPose(curPose, targets[n].tLastUpdate);

            // if feasibility checking enabled, test
            int nTraj = targets[n].libraryPtr->getNumTraj();
            vector<int> feasibleIndex;
            if (state.isEdtExist){
                lib::FeasibilityParam testingParam = getFeasibilityTestingParamBase();
                targets[n].libraryPtr->registerFeasibility(testingParam,feasibleIndex);
                if (feasibleIndex.empty()) {
                    ROS_ERROR("no feasible prediction for target %d. Aborting prediction", n);
                    return false;
                }
            } else // do not check feasibility
                for (int trajIdx = 0; trajIdx < nTraj ; trajIdx ++)
                    feasibleIndex.push_back(trajIdx);

            // get the best among feasible prediction
            int trajBest = 0;
            float costMin = numeric_limits<float>::max();
            auto observationQueue = targets[n].observationRawQueue ;
            int nObservation  = observationQueue.size();
            for (int trajIdx : feasibleIndex) {
                // cost 1: observation error
                float errorSum = 0;
                for (auto observation : observationQueue ) {
                    ros::Time t = observation.first;
                    Point pnt = observation.second;
                    errorSum +=  targets[n].libraryPtr->eval(trajIdx,t).distTo(pnt);
                }
                errorSum /= float(nObservation);


                // cost 2: diff with the previous prediction over shared time interval
                float diffSum = 0;
                if (targets[n].isPredicted) {
                    // shared time knots between previous and current
                    int nPointInSharedInterval = 6;
                    ros::Time t00 = state.tLastPrediction, t0f = t00 + ros::Duration(param.horizon);
                    ros::Time t10 = predictionStartTime, t1f = t10 + ros::Duration(param.horizon);
                    float sharedDuration = (t0f - t10).toSec();
                    float dt = sharedDuration / float(nPointInSharedInterval - 1);
                    for (int pntIdx = 0; pntIdx < nPointInSharedInterval; pntIdx++) {
                        ros::Time timePnt = t10 + ros::Duration(dt * pntIdx);
                        Point evalPreviousPrediction = targets[n].curPrediction.eval(timePnt);
                        Point evalCurPrediction = targets[n].libraryPtr->eval(trajIdx,timePnt);
                        diffSum += evalCurPrediction.distTo(evalPreviousPrediction);
                    }
                    diffSum /= nPointInSharedInterval;
                }

                // register the costs
                float cost = errorSum + param.weightOnShapeChange * diffSum;
                targets[n].libraryPtr->registerCost(trajIdx,cost);
                if (costMin > cost ){
                    trajBest = trajIdx;
                    costMin = cost;
                }
            }

            ROS_INFO("TargetManger: for target %d, picked %d th traj, feasible = [%d/%d]",
                     n,trajBest,feasibleIndex.size(), nTraj);

            // update prediction
            Traj predictionTraj;
            ros::Time useTime; // this is same with tLastUpdate as this is the library ref tim e
            targets[n].libraryPtr->getTraj(trajBest,predictionTraj,useTime);
            targets[n].curPrediction = PredictionOutput(targets[n].tLastUpdate,predictionTraj);
            targets[n].feasiblePredictionMarker =
                    targets[n].libraryPtr->getMarkerFeasibleTraj(targets[n].getLibRefFrameName());
            state.predictionAccumErrors[n] = 0.0;
            state.tLastPrediction = predictionStartTime;
            targets[n].isPredicted = true;

            predictionOutputSet.push_back(targets[n].curPrediction);

        }
        double elapse = timer.stop();
        ROS_INFO("TargetManager: prediction took %f ms totally", elapse );
        mutex_.unlock();
        return true;
    }
    /**
     * Return target poses by looking-up tf when this function is called.
     * @return if no tf, just return the last pose at queue
     */
    vector<Pose> TargetManager::lookupLastTargets() const {
        vector<Pose> poseSet(param.nTarget);

        for (int n = 0 ; n < param.nTarget ; n++) {
            tf::StampedTransform stampedTransform;
            stampedTransform.setIdentity();
            try {
                // setting ros::Time::now for looking up cannot retrieve enough information in the buffer for this case
                tfListener->lookupTransform(param.worldFrameId, param.targetObservationFrameId[n],
                                            ros::Time(0), stampedTransform);
                poseSet[n] = Pose(stampedTransform);
            } catch (tf::TransformException &ex) {
                ROS_WARN("TargetManager: %s not found. will return just last pose from timer callback",
                         param.targetObservationFrameId[n].c_str());
                poseSet[n] = targets[n].curPose;
//                ROS_INFO_STREAM(ex.what());
            }
        }
        return poseSet;
    }

    /**
     * Based on state, returns triggering condition
     * @return true: you need to predict / false: dont need
     * @bug to use accumulated prediction error condition, mainCallback should run from global callback spinning
     */
    bool TargetManager::needPrediction() {

        // condition 1 : expire of prediction
        bool cond1 = (ros::Time::now() - state.tLastPrediction).toSec() > param.predictionAgeTol;
        if (cond1){
            ROS_INFO("TargetManager: prediction too old %f. need new prediction.",
                     (ros::Time::now() - state.tLastPrediction).toSec());
            return true;
        }

        // condition 2: inaccuracy of prediction
        bool cond2;
        for (int n =0 ; n < param.nTarget ; n++) {
            cond2 = (state.predictionAccumErrors[n] > param.accumErrorTol);
            if (cond2) {
                ROS_INFO("TargetManager: prediction accum. err. =  %f > %f for target %d."
                         " need new prediction.",
                         state.predictionAccumErrors[n], param.accumErrorTol, n);
                return true;
            }
        }

        // condition 3 : if prediction traj collide with obstacle
        bool cond3;
        for (int n = 0 ; n < param.nTarget ; n++){
            lib::FeasibilityParam testingParam = getFeasibilityTestingParamBase();
            testingParam.margin = param.predictionRiskTol;
            cond3 = not targets[n].libraryPtr->checkTrajectoryFeasibility(testingParam,targets[n].curBestPick);
            if (cond3) {
                ROS_INFO("TargetManager: prediction for target %d will collide with obstacle."
                         " need new prediction.",n);
                return true;
            }
        }

        // else
        return false;
    }

}
