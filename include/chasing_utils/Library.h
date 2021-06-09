//
// Created by jbs on 21. 5. 11..
//

#ifndef AUTO_CHASER4_LIBRARY_H
#define AUTO_CHASER4_LIBRARY_H
#include <chasing_utils/Utils.h>

namespace chasing_utils {

    namespace lib {

        /**
         * Generation model for polynomial array of library
         */
        enum Strategy {
            DISCRETE_ACCEL_INTEGRATOR,
            SKELETON_PERMUTATION
        };

        struct LibraryParam{

            LibraryParam() {};
            LibraryParam(const ros::NodeHandle& nhTarget); // Parse from ros node handle

            float horizon = 1.0;
            bool verbose = true;
            string libFolder = "";
            Strategy strategy = Strategy::DISCRETE_ACCEL_INTEGRATOR;

            // TraverseGrid (important)
            float TG_resolution = 0.2;
            float TG_padding= 0.3; // No need to be large if include Points are enough
            float TG_inspectionDx = 0.2; // Stride along primary trajectory
            float TG_inspectionDy = 0.2; // Stride along bearing vector (only active in Skeleton Permutation)

            //! offline
            bool isLoadMode = true;
            PolynomialXYZ* polyRefPtr; //! active in PERMUTATION STRATEGY
            //! saveLib = true, save binary file: saveFolder/myName. In load mode vice versa.
            bool saveLib = true;

            // Discrete Accel Integration (DI)
            // TODO currently only 2D is supported
            int DI_speedDisc = 4;
            float DI_speedMin  = 0.0;
            float DI_speedMax = 2.5;
            float DI_accelXDisc = 5;
            float DI_accelXMin = -1;
            float DI_accelXMax = 1;
            float DI_accelYDisc = 5;
            float DI_accelYMin = -1;
            float DI_accelYMax = 1;

            //! online
            std_msgs::ColorRGBA VIS_colorFeasibleTraj;
            float VIS_widthFeasibleTraj;
            int VIS_strideFeasibleTraj;

            //! visualizations
            float VIS_drawStartTime = 0;

            std_msgs::ColorRGBA VIS_colorRefTraj;
            float VIS_widthRefTraj;

            std_msgs::ColorRGBA VIS_colorCandidTraj;
            float VIS_widthCandidTraj;
            int VIS_strideCandidTraj;

            std_msgs::ColorRGBA VIS_travAssociation;
            std_msgs::ColorRGBA VIS_travNonAssociation;

        };


        /**
         * 4D boolean array [nx*ny*nz*nTraj] to encode the association between the 3D polynomial array and traverse.
         * If an element = true, the traj traverse (x,y,z) cell.
         * The ref-frame of trajectory generation is origin and x-forward
         */
        class Library {
        protected:
            LibraryParam param; //! includes paramteres for generation
            int index; //! name tag. optinal
            string getMyName() const {return "lib_" + to_string(index);};
            PolynomialXYZ polyRef;  //! In case of skeleton-permutation, this is the base traj
            PolynomialXYZ *polyArr; //! Total candidate trajectory
            int nTraj; //! number of trajectory of polyArr
            TraverseGrid *traverseGridPtr;
            vector<int> associationLinearInd; //! linear index of (x,y,z,n) s.t. TraverseGrid value = true
            vector<gridTrajPair> gridTrajPairSet; //! list of <(i,j,k), its trajectory list in association> s.t. association > 0
            vector<trajGridPair> trajGridPairSet; //! list of <traj, its cell list in association> s.t. association > 0

            //! build polyArr from strategy
            PointSet generateTrajectory();
            PointSet getInspectionPoints(const PolynomialXYZ& poly) const;

        public:
            Library(LibraryParam param, int index = 0 );
            int getNumTraj() const {return nTraj;}
            float getHorizon () const {return param.horizon;}

            //! Returns markers w.r.t refFrame (this might not be static on-the-fly)
            visualization_msgs::MarkerArray getMarkerCandidTraj(string frameId) const; // stride of parameter
            visualization_msgs::MarkerArray getMarkerCandidTraj(string frameId, const vector<int>& trajIdx ) const;
            visualization_msgs::MarkerArray getMarkerCandidTraj(string frameId, const vector<int>& trajIdx,
                                                                const vector<std_msgs::ColorRGBA>& colors ) const;
            visualization_msgs::Marker getMarkerRefTraj(string frameId) const;
            visualization_msgs::Marker getMarkerTraverseGrid(string frameId) const;
            visualization_msgs::MarkerArray getMarkerTotal(string frameId) const;
            pcl::PointCloud<pcl::PointXYZI> getMarkerAssociation(string frameId) const {return traverseGridPtr->visualizeAll( frameId);};

            PolynomialXYZ* getPolyPtrAt(int trajIdx) const {return &polyArr[trajIdx];}
            Traj getPolyTrajAt(int trajIdx,float drawStartTime = 0.0) const {return polyArr[trajIdx].toTraj(drawStartTime,param.horizon,0.03f);}

        };


        /**
         * Parameters for testing either obstacle collision or occlusion
         */
        struct FeasibilityParam{
            octomap_server::EdtOctomapServer* edtServerPtr;
            float margin = 0.4;
        };

        /**
         * Library class to perform online feasibility testing
         */
        class LibraryOnline : public Library{
        private:
            Pose libRefPose; // transform from world coordinate to library develop frame
            ros::Time libRefTime; // library develop time. eval time on the poly = (ros::Time - libRefTime).toSec()
            vector<float> costs; // cost of trajectory in library
            vector<bool> feasibility; // feasibility of trajectory in library
            vector<int> feasibleTraj;
            pcl::PointCloud<pcl::PointXYZI> cellMarker; // encode edt value on traverse grid

        public:
            LibraryOnline( LibraryParam param, int index ) ;
            void updateLibRefPose(const Pose& T_wl,ros::Time refTime) { libRefTime = refTime; libRefPose = T_wl; };
            bool registerFeasibility (FeasibilityParam feasibilityParam, vector<int>& feasibleIndex);
            void registerCost(int trajIdx, float cost);
            bool checkTrajectoryFeasibility (FeasibilityParam feasibilityParam,int trajIdx) const;

            visualization_msgs::MarkerArray getMarkerFeasibleTraj(string frameId) const;
            pcl::PointCloud<pcl::PointXYZI> getMarkerCells(string worldFrameId) const;
            Point eval(int trajIdx, ros::Time evalTime) const ;

            Pose getRefPose() const {return libRefPose; }
            void getTraj(int trajIdx, Traj & traj, ros::Time& refTime) const;

        };

    }
}
#endif //AUTO_CHASER4_LIBRARY_H
