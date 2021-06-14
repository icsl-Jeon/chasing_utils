//
// Created by jbs on 21. 6. 7..
//

#ifndef CHASING_UTILS_TARGETMANAGER_H
#define CHASING_UTILS_TARGETMANAGER_H
#include <chasing_utils/Utils.h>
#include <chasing_utils/Library.h>
#include <octomap_server/EdtOctomapServer.h>

using namespace std;
namespace chasing_utils {

    typedef std::pair<ros::Time,Point> Observation; //! (tobs, Pobs)

    struct PredictionOutput{
    private:
        Traj predictionTraj;
        ros::Time refTime;
    public:
        PredictionOutput() = default;;
        PredictionOutput(ros::Time refTime, Traj predictionTraj):
            refTime(refTime), predictionTraj(predictionTraj) {};
        Point eval(ros::Time evalTime) const;
        ros::Time getRefTime() const {return refTime; }
        Traj getTraj() const {return predictionTraj; }
    };

    struct Target {
        bool isPredicted = false;
        bool isPoseUpdate = false;
        int myIndex = 0;
        ros::Time tLastUpdate;
        deque<Observation> observationRawQueue; //! raw observations
        PredictionOutput curPrediction;
        Pose curPose;

        lib::LibraryOnline* libraryPtr;
        int curBestPick = 0;
        visualization_msgs::MarkerArray feasiblePredictionMarker;

        pcl::PointCloud<pcl::PointXYZI> visualizeObservation(string worldFrameId) const;
        nav_msgs::Path visualizePrediction(string worldFrameId) const;
        visualization_msgs::MarkerArray visualizeFeasibleSet() const;
        string getLibRefFrameName() const {return "lib_"+to_string(myIndex);};

    };

    class TargetManager {
        struct Param{
            int nTarget = 2;
            float observationInterval = 0.2;
            int observationQueueSize = 20; // # of observations used for prediction
            int nMinObservationForPrediction = 3;
            vector<string> targetObservationFrameId;
            string worldFrameId = "map";
            float predictionCollisionMargin = 0.3;
            float weightOnShapeChange = 0.2;

            float accumErrorTol = 10;
            float predictionAgeTol = 1.0;
            float predictionRiskTol = 0.5;

            float horizon; // this is read-only from library object

            Param() = default;
            void parseFrom(const ros::NodeHandle& nhTarget);
        };

        struct State{
            bool isEdtExist = false;
            vector<float> predictionAccumErrors; // reset at prediction
            ros::Time tLastPrediction = ros::Time(0.0);
        };
        struct PublisherSet{
            vector<ros::Publisher> observationRawQueueSet;
            vector<ros::Publisher> predictionSet;
            vector<ros::Publisher> curPredictionPointSet;
            vector<ros::Publisher> curFeasibleCandidateSet;
            vector<ros::Publisher> curPredictionErrorSet;
            vector<ros::Publisher> curTraverseGridSet;
        };

    private:
        mutex mutex_;
        string myName = "TargetManager";
        vector<Target> targets; // single or dual
        Param param;
        State state;
        octomap_server::EdtOctomapServer* edtServerPtr;

        ros::NodeHandle nh;
        ros::NodeHandle nhTimer;

        ros::Timer observationTimerCaller;
        ros::AsyncSpinner* asyncSpinnerPtr;
        ros::CallbackQueue observationCallbackQueue;
        ros::Timer mainTimerCaller;
        tf::TransformListener* tfListener;
        tf::TransformBroadcaster* tfBroadcaster;
        PublisherSet pubSet;

        lib::FeasibilityParam getFeasibilityTestingParamBase();
        void observationTimerCallback(const ros::TimerEvent& event);
        void mainTimerCallback(const ros::TimerEvent& event);
        void initROS();
        void updateObservation(int targetIdx,
                               const tf::StampedTransform& stampedTransform);

    public:
        TargetManager(octomap_server::EdtOctomapServer* edtServerPtr = NULL);
        bool predict(vector<PredictionOutput>& predictionOutputSet);
        vector<Pose> lookupLastTargets() const;
        bool needPrediction();
    };


}




#endif //CHASING_UTILS_TARGETMANAGER_H
