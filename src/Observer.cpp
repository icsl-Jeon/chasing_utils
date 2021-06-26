//
// Created by jbs on 21. 3. 21..
//

#include <chasing_utils/Observer.h>

namespace chasing_utils {

    namespace observer {

        Observer::Observer(Pose pose, ObserverTag tag) : pose(pose), tag(tag) {}

        ScoreTriplet Observer::getScoreTripletRaw() const {
            return scoreRaw;
        }

        float ObserverSet::getScoreNormalizedWeightSum(int m) const {
            ScoreTriplet triplet = getScoreNormalized(m);
            return param.scoreWeightSAC * triplet.sac +
                   param.scoreWeightVAA * triplet.vaa +
                   param.scoreWeightVAO * triplet.vao;
        }

/**
 * Set of Observers surrounding the target at step n
 * The configuration of their poses are defined by same parameters along steps
 * @param n time step of this set
 * @param target
 */
        ObserverSet::ObserverSet(ObserverParam param, int n, Point target) : param(param) {

            PoseSetInitConfig initParam = param.getPoseInitConfig();
            nObserverAlongRay = initParam.step[0];

            int observerIdx = 0;
            for (int d2 = 0; d2 < initParam.step[1]; d2++) //! per azim
                for (int d3 = 0; d3 < initParam.step[2]; d3++) { // ! per elev
                    Point dir;
                    dir.x = cos(initParam.val(2, d3)) * cos(initParam.val(1, d2));
                    dir.y = cos(initParam.val(2, d3)) * sin(initParam.val(1, d2));
                    dir.z = sin(initParam.val(2, d3));

                    // save the set of ray direction and innermost memeber
                    directionSet.points.push_back(dir);
                    directionInnerIndex.push_back(observerIdx);

                    // ! stride along ray
                    for (int d1 = 0; d1 < initParam.step[0]; d1++) {
                        Point camLoc = target + dir * initParam.val(0, d1);
                        Pose camPose = Pose(camLoc, target);
                        observerSet.emplace_back(Observer(camPose, make_pair(n, observerIdx)));
                        observerIdx++;
                    }
                }
        }


/**
 * This should be called after all observers are assigned.
 * For zero scores, let's not include
 * @param m
 * @return
 */
        ScoreTriplet ObserverSet::getScoreNormalized(int m) const {

            // check if all scores are available for the Observer in this set
            bool isAllScoreOkay = true;
            for (auto it : observerSet)
                isAllScoreOkay = isAllScoreOkay and it.isAllScoreAssigned();

            assert(isAllScoreOkay && "Normalized score query should be called after all scores of observers computed ");

            pair<ScoreTriplet, ScoreTriplet> minMax = getMaxMinScore();
            auto minScoreTriplet = minMax.first;
            auto maxScoreTriplet = minMax.second;

            ScoreTriplet scores = observerSet[m].getScoreTripletRaw();
            scores.sac = (scores.sac - minScoreTriplet.sac) / (maxScoreTriplet.sac - minScoreTriplet.sac);
            scores.vao = (scores.vao - minScoreTriplet.vao) / (maxScoreTriplet.vao - minScoreTriplet.vao);
            scores.vaa = (scores.vaa - minScoreTriplet.vaa) / (maxScoreTriplet.vaa - minScoreTriplet.vaa);
            return scores;
        }

        pair<ScoreTriplet, ScoreTriplet> ObserverSet::getMaxMinScore() const {

            ScoreTriplet maxScoreTriplet; // max scores in the observer
            ScoreTriplet minScoreTriplet; // max scores in the observer

            float maxSAC = numeric_limits<float>::lowest();
            float maxVAO = numeric_limits<float>::lowest();
            float maxVAA = numeric_limits<float>::lowest();

            float minSAC = numeric_limits<float>::max();
            float minVAO = numeric_limits<float>::max();
            float minVAA = numeric_limits<float>::max();

            for (auto it = observerSet.begin(); it < observerSet.end(); it++) {

                if (it->getScoreTripletRaw().sac > maxSAC)
                    maxSAC = it->getScoreTripletRaw().sac;
                if (it->getScoreTripletRaw().vaa > maxVAA)
                    maxVAA = it->getScoreTripletRaw().vaa;
                if (it->getScoreTripletRaw().vao > maxVAO)
                    maxVAO = it->getScoreTripletRaw().vao;

                if (it->getScoreTripletRaw().sac < minSAC)
                    minSAC = it->getScoreTripletRaw().sac;
                if (it->getScoreTripletRaw().vaa < minVAA)
                    minVAA = it->getScoreTripletRaw().vaa;
                if (it->getScoreTripletRaw().vao < minVAO)
                    minVAO = it->getScoreTripletRaw().vao;
            }

            maxScoreTriplet.sac = maxSAC;
            maxScoreTriplet.vaa = maxVAA;
            maxScoreTriplet.vao = maxVAO;

            minScoreTriplet.sac = minSAC;
            minScoreTriplet.vaa = minVAA;
            minScoreTriplet.vao = minVAO;
            return make_pair(minScoreTriplet, maxScoreTriplet);
        }

        float ObserverSet::getCost(int m) const {
            ScoreTriplet triplet = getScoreNormalized(m);
            return param.scoreWeightSAC * (1 - triplet.sac) +
                   param.scoreWeightVAA * (1 - triplet.vaa) +
                   param.scoreWeightVAO * (1 - triplet.vao);
        }

/**
 * intensity time coloring for observer point
 * @param setIntensityWithTime
 * @return
 */
        PointXYZI Observer::toPntXYZI() const {
            Point pnt = pose.getTranslation();
            PointXYZI pclPnt;
            pclPnt.x = pnt.x;
            pclPnt.y = pnt.y;
            pclPnt.z = pnt.z;
            pclPnt.intensity = (float) tag.first;
            return pclPnt;
        }

/**
 * get iterRange per ray direction.
 * @return [n] element = observers referance iterator along n th ray
 */
        vector<IterObserverRange> ObserverSet::getObserverRefAlongRaySet() {
            vector<IterObserverRange> iterPairAlongRaySet;
            vector<Observer> &observers = observerSet;
            for (int rayIdx = 0; rayIdx < directionSet.size(); rayIdx++) {
                int idxStart = rayIdx * nObserverAlongRay, idxEnd = (rayIdx + 1) * nObserverAlongRay;
                iterPairAlongRaySet.emplace_back(observers.begin() + idxStart, observers.begin() + idxEnd);
            }
            return iterPairAlongRaySet;
        }

        vector<PointXYZI> ObserverSet::toPntXYZI() const {
            vector<PointXYZI> pclPoints;
            for (auto it = observerSet.begin(); it != observerSet.end(); it++) {
                pclPoints.push_back(it->toPntXYZI());
            }
            return pclPoints;
        }

        void ObserverSet::setScoreRaw(int m, int d, float score) {
            switch (d) {
                case 0:
                    observerSet[m].setScoreSAC(score);
                    return;
                case 1:
                    observerSet[m].setScoreVAO(score);
                    return;
                case 2:
                    observerSet[m].setScoreVAA(score);
                    return;
            }
        }


/**
 * Spawn observerSet along baseTraj at kKnots times
 * @param param
 * @param baseChain (t1,p1) , (t2,p2) , (t3,p3) ,...,(tN,pN)
 */
        OSC::OSC(ObserverParam param, Traj baseChain) : baseChain(baseChain), param(param) {
            curChainSize = baseChain.getSize();
            for (int n = 0; n < curChainSize; n++) {
                float t = baseChain.ts[n];
                Point target = baseChain.eval(t);
                chain.emplace_back(ObserverSet(param, n, target));
                camSizePerStep = chain[n].size(); // redundant, same for all time step
            }
        }

/**
 * Min intensity : n=0
 * @return
 */
        PointCloud<PointXYZI> OSC::getTimeColoredPoints() const {

            PointCloud<PointXYZI> pcl;
            for (int n = 0; n < curChainSize; n++) {
                auto pclSet = chain[n].toPntXYZI();
                pcl.insert(pcl.end(), pclSet.begin(), pclSet.end());
            }
            return pcl;
        }

        PointCloud<PointXYZI> OSC::getBasePoints() const {

            PointCloud<PointXYZI> pcl;
            for (int n = 0; n < curChainSize; n++) {
                PointXYZI point;
                point.x = baseChain.points[n].x;
                point.y = baseChain.points[n].y;
                point.z = baseChain.points[n].z;
                point.intensity = n;
                pcl.push_back(point);
            }
            return pcl;
        }


        visualization_msgs::Marker OSC::getSphereList(string worldFrameId, string ns, int id) const {
            visualization_msgs::Marker marker;
            marker.ns = ns;
            marker.id = id;
            marker.type = visualization_msgs::Marker::SPHERE_LIST;
            marker.header.frame_id = worldFrameId;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = param.OL_visOscRadius;
            marker.scale.y = param.OL_visOscRadius;
            marker.scale.z = param.OL_visOscRadius;
            marker.color = param.OL_visOscColor;

            for (const auto &os: chain) {
                IterConstObserverRange iterRange;
                for (auto it = iterRange.first; it != iterRange.second; it++)
                    marker.points.push_back(it->getLocation().toGeometry());
            }
            return marker;
        }


/**
 * get pose of all the observers in OSC
 * @return
 */
        PoseSet OSC::getAllPoseSet() const {
            PoseSet poseSet;
            for (int n = 0; n < curChainSize; n++)
                std::transform(chain[n].getConstIterator().first, chain[n].getConstIterator().second,
                               std::back_inserter(poseSet.poseSet),
                               [](const Observer &observer) -> Pose { return observer.getPose(); });
            return poseSet;
        }

        PointSet OSC::getAllPointSet() const {
            PointSet pntSet;

            for (int n = 0; n < curChainSize; n++)
                std::transform(chain[n].getConstIterator().first, chain[n].getConstIterator().second,
                               std::back_inserter(pntSet.points),
                               [](const Observer &observer) -> Point { return observer.getLocation(); });
            return pntSet;
        }

        pair<PoseSet, vector<ObserverTag>> OSC::getAllPoseSetWithTag() const {

            PoseSet poseSet;
            vector<ObserverTag> tagSet;

            for (int n = 0; n < curChainSize; n++) {
                std::transform(chain[n].getConstIterator().first, chain[n].getConstIterator().second,
                               std::back_inserter(poseSet.poseSet),
                               [](const Observer &observer) -> Pose { return observer.getPose(); });

                std::transform(chain[n].getConstIterator().first, chain[n].getConstIterator().second,
                               std::back_inserter(tagSet),
                               [](const Observer &observer) -> ObserverTag { return observer.getTag(); });

            }

            return make_pair(poseSet, tagSet);

        }

/**
 * Creat point chain at (t1,...tN) by permuting ObserverSet along time step.
 * arr = [nStep * nTraj] where arr[i*nStep : (i+1)*nStep] is point chain of ith trajectory
 * @param xArr
 * @param yArr
 * @param zArr
 * @return ObserverTargetChain[nTraj]
 */
        vector<ObserverTagChain> OSC::getPermutationPointChain(double *&xArr, double *&yArr, double *&zArr) const {

            int nTraj = pow(camSizePerStep, curChainSize);
            vector<ObserverTagChain> vecTagArr(nTraj);

            Permutator permt(curChainSize, camSizePerStep);
            int *indexSet = permt.getOutputArr();

            xArr = new double[curChainSize * nTraj];
            yArr = new double[curChainSize * nTraj];
            zArr = new double[curChainSize * nTraj];

            for (int i = 0; i < nTraj; i++) {
                ObserverTagChain tagArr(curChainSize);
                for (int n = 0; n < curChainSize; n++) {
                    uint m = indexSet[curChainSize * i + n];
                    Point point = getObserver(n, m).getLocation();
                    xArr[i * curChainSize + n] = point.x;
                    yArr[i * curChainSize + n] = point.y;
                    zArr[i * curChainSize + n] = point.z;
                    tagArr[n].first = n;
                    tagArr[n].second = m;
                }
                vecTagArr[i] = tagArr;
            }
            return vecTagArr;
        }

/**
 *
 * @param d 0 = sac / 1 = vao / 2 = vaa
 * @param score
 */
        void OSC::setScoreRaw(ObserverTag tag, int d, float score) {
            chain[tag.first].setScoreRaw(tag.second, d, score);
        }


/**
 * transform observer poses and base trajectory p_{new} = T01 p_{orig}
 * @param T01
 * @bug score is not updated
 */
        void OSC::applyTransform(Pose T01) {
            baseChain.applyTransform(T01);

            for (int n = 0; n < curChainSize; n++) {
                for (auto it = chain[n].getRefIterator().first;
                     it < chain[n].getRefIterator().second; it++)
                    it->applyTransform(T01);
            }
        }

/**
 * spawn Observers around baseTraj
 * @param param
 * @param baseTraj t = [0,param.horizon]
 * @param t0,tf = in [0,param.horizon]
 * @param dt time interval to spawn OS
 * @param maxStep max number of knots
 * @return
 */
        OSC spawnOscAroundTraj(ObserverParam param, Traj baseTraj,
                               float t0, float tf, float dt, int maxStep) {
            int Nstep = max(min((int) ceil(baseTraj.get_length(t0, tf) / dt),
                                maxStep), 1);

            VectorXf tKnots(Nstep);
            tKnots.setLinSpaced(Nstep, t0, tf);
            vector<float> ts(tKnots.data(), tKnots.data() + Nstep);
            vector<Point> points;
            for (auto t : ts) {
                points.push_back(baseTraj.eval(t));
            }
            Traj baseChain(points, ts);
            return OSC(param, baseChain);
        }

/**
 * Get Direction set of nth ObserverSet
 * @param n
 * @return
 */
        PointSet OSC::getDirectionAlongRaySet(int n) const {
            return chain[n].getDirectionAlongRaySet();
        }

/**
 * Get normalized score triplet (SAC / VAO / VAA). This should be called after all scores are assigned
 * @param tag
 * @return
 */
        ScoreTriplet OSC::getNormalizedScore(ObserverTag tag) const {
            int n = tag.first;
            int m = tag.second;
            return chain[n].getScoreNormalized(m);
        }
    }
}