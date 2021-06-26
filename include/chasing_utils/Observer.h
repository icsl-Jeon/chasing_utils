//
// Created by jbs on 21. 3. 21..
//

#ifndef AUTO_CHASER2_OBSERVER_H
#define AUTO_CHASER2_OBSERVER_H

#include <chasing_utils/Utils.h>

#define INFEASIBLE_SCORE 0.0
#define INFEASIBLE_SCORE_NORM -1.0
using namespace pcl;


namespace chasing_utils {
    namespace observer {
        struct ObserverParam{

            float scoreWeightSAC = 1.0;
            float scoreWeightVAO = 1.0;
            float scoreWeightVAA = 1.0;

            float discLevel = 0.2;
            float oscGenStartTime = 0.3;
            int nMaxStep; // maximum number of step

            int azimStep;

            float radMin;
            float radMax;
            int radStep;

            float elevMin;
            float elevMax;
            int elevStep;

            std_msgs::ColorRGBA OL_visOscColor;
            float OL_visOscRadius;

            PoseSetInitConfig getPoseInitConfig() {
                PoseSetInitConfig setConfig;
                setConfig.step[0] = radStep;
                setConfig.axisMin[0] = radMin;
                setConfig.axisMax[0] = radMax; // radius
                setConfig.step[1] = azimStep;
                setConfig.axisMin[1] = 0.0;
                setConfig.axisMax[1] = 2 * M_PI * (setConfig.step[1] - 1) / setConfig.step[1];   // azimuth
                setConfig.step[2] = elevStep;
                setConfig.axisMin[2] = elevMin;
                setConfig.axisMax[2] = elevMax; // elevation
                return setConfig;
            }


        };

        struct ScoreTriplet {
            float sac = INFEASIBLE_SCORE;
            float vao = INFEASIBLE_SCORE;
            float vaa = 0.0;
        };

        class OSC;

        typedef pair<int, int> ObserverTag; //(n,m)
        typedef vector<ObserverTag> ObserverTagChain; // (0,m_0),(1,m_1),.. (n,m_n)

        class Observer {
            friend class ObserverSet;

        private:
            ObserverTag tag;
            Pose pose;
            ScoreTriplet scoreRaw; //! raw value of scores
            bool isSacKnown = false; //! was edf calculated based on known cell
            bool isVaoExact = false; //! were all the cells known along the ray until the observer
            bool isScoreAssigned[3] = {false, false, false};

            string getNameTag() const { return to_string(tag.first) + "_" + to_string(tag.second); }

            void setScoreSAC(float score) {
                scoreRaw.sac = score;
                isScoreAssigned[0] = true;
            }

            void setScoreVAO(float score) {
                scoreRaw.vao = score;
                isScoreAssigned[1] = true;
            }

            void setScoreVAA(float score) {
                scoreRaw.vaa = score;
                isScoreAssigned[2] = true;
            }

        public:
            Observer(Pose pose, ObserverTag tag);

            ObserverTag getTag() const { return tag; }

            PointXYZI toPntXYZI() const;

            ScoreTriplet getScoreTripletRaw() const;

            Point getLocation() const { return pose.getTranslation(); }

            Pose getPose() const { return pose; };

            void applyTransform(Pose T01) { pose.applyTransform(T01); };

            bool isAllScoreAssigned() const {
                return isScoreAssigned[0] and
                       isScoreAssigned[1] and isScoreAssigned[2];
            }
        };


        typedef pair<vector<Observer>::const_iterator, vector<Observer>::const_iterator>
                IterConstObserverRange;

        typedef pair<vector<Observer>::iterator, vector<Observer>::iterator>
                IterObserverRange;

        class ObserverSet {
        private:
            ObserverParam param;
            vector<Observer> observerSet;
            PointSet directionSet; //! direction defined by azim,elev pair
            vector<int> directionInnerIndex; //! index of Observer having the minimum radius from target for a direction
            int nObserverAlongRay; //! = radius step

        public:
            ObserverSet(ObserverParam param, int n, Point target); // make surrounding ObserverSet on target
            int size() const { return observerSet.size(); }

            pair<ScoreTriplet, ScoreTriplet> getMaxMinScore() const;

            ScoreTriplet getScoreNormalized(int m) const;

            float getScoreNormalizedWeightSum(int m) const;

            Point getPoint(int m) const { return observerSet[m].getLocation(); }

            Pose getPose(int m) const { return observerSet[m].getPose(); }

            float getCost(int m) const;

            void setScoreRaw(int m, int d, float score);

            vector<PointXYZI> toPntXYZI() const;

            IterConstObserverRange getConstIterator() const {
                return make_pair(observerSet.cbegin(), observerSet.cend());
            }

            IterObserverRange getRefIterator() { return make_pair(observerSet.begin(), observerSet.end()); }

            vector<IterObserverRange> getObserverRefAlongRaySet();

            PointSet getDirectionAlongRaySet() const { return directionSet; }

        };


        /**
         * @brief Observer Set Chain
         */
        class OSC {
        private:
            bool isVaoAssigned = false;
            bool isVaaAssigned = false;

            unsigned int curChainSize;
            int camSizePerStep;

            ObserverParam param;
            vector<ObserverSet> chain;
            Traj baseChain;


        public:
            //! Only positions of observers will be created here
            OSC(ObserverParam param, Traj baseChain);

            /**
             * Core service functions
             */
            void setScoreRaw(ObserverTag tag, int d, float score);

            //! make chains by choosing a points from observerSet per time
            vector<ObserverTagChain> getPermutationPointChain(
                    double *&xArr, double *&yArr, double *&zArr) const;

            /**
             * Set and get service functions
             */
            Pose getPose(int n, int m) const { return chain[n].getPose(m); }

            Observer getObserver(int n, int m) const { return *(chain[n].getConstIterator().first + m); }

            IterConstObserverRange getAllObserver(int n) const { return chain[n].getConstIterator(); }

            PointCloud<PointXYZI> getTimeColoredPoints() const;

            PointCloud<PointXYZI> getBasePoints() const;

            visualization_msgs::Marker
            getSphereList(string worldFrameId, string ns = "", int id = 0) const; //! visualization_msgs marker sphere list
            PoseSet getAllPoseSet() const;

            pair<PoseSet, vector<ObserverTag>> getAllPoseSetWithTag() const;

            PointSet getAllPointSet() const;

            int getCurChainSize() const { return chain.size(); };

            int getCurCamSizePerStep() const { return camSizePerStep; }

            PointSet getDirectionAlongRaySet(int n) const;

            Point getBasePoint(int n) const {
                assert (n >= 0 and n < curChainSize);
                return baseChain.points[n];
            }

            Traj getBaseChain() const {
                return baseChain;
            }

            ScoreTriplet getNormalizedScore(ObserverTag tag) const;

            vector<IterObserverRange> getObserverRefAlongRaySet(int n) { return chain[n].getObserverRefAlongRaySet(); }

            void applyTransform(Pose T01);
        };

        //! Generate OSC from making baseChain by discretization of baseTraj
        OSC spawnOscAroundTraj(ObserverParam param, Traj baseTraj,
                               float t0,float tf,float dt,int maxStep);
    }

}
#endif //AUTO_CHASER2_EVALUATOR_H
