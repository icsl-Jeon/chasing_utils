//
// Created by jbs on 21. 3. 14..
//

#ifndef CHASING_UTILS_H
#define CHASING_UTILS_H

#include <chrono>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <mutex>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/PolygonStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/point_cloud2_iterator.h>


#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/publisher.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>

#include <chasing_utils/progressbar.hpp>

#include <octomap_server/TrackingOctomapServer.h>
#include <octomap_server/EdtOctomapServer.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap/octomap_types.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>


#define TABLE_CELL_WIDTH 12
#define TABLE_TAG_WIDTH 10
#define PRECISION 2

#define FILL_CELL_LEFT(S) std::cout<< std::setfill(' ') << std::left << std::setw(TABLE_CELL_WIDTH) << std::setprecision(PRECISION)<<S;
#define FILL_TAG(S) std::cout<< std::setfill(' ') << std::left << std::setw(TABLE_TAG_WIDTH) << S;
#define FILL_CELL_RIGHT(S) std::cout<< std::setfill(' ') << std::right << std::setw(TABLE_CELL_WIDTH) << std::setprecision(PRECISION)<< S;
#define TOP_RULE_STAR(W) std::cout << std::endl; std::cout << std::setfill('*') << std::setw(TABLE_CELL_WIDTH * (W) + TABLE_TAG_WIDTH) << "*" <<std::endl;
#define MID_RULE_DASH(W) std::cout << std::endl; std::cout << std::setfill('-') << std::setw(TABLE_CELL_WIDTH * (W) + TABLE_TAG_WIDTH) << "-" <<std::endl;
#define BOTTOM_RULE_EQ(W)  std::cout << std::setfill('=') << std::setw(TABLE_CELL_WIDTH * (W) + TABLE_TAG_WIDTH) << "=" <<std::endl;
#define TITLE(W,S) std::cout << std::setfill(' ') << std::left << std::setw(TABLE_CELL_WIDTH * (W) + TABLE_TAG_WIDTH) << S;
#define NEW_LINE std::cout << std::endl;

using namespace std::chrono;
using namespace std;
using namespace Eigen;

#define DIM 3

namespace chasing_utils {

    class Timer{
        steady_clock::time_point t0;
        double measuredTime;
    public:
        Timer(){t0 = steady_clock::now();}
        double stop(bool isMillisecond = true) {
            if (isMillisecond)
                measuredTime =  duration_cast<milliseconds>(steady_clock::now() - t0).count();
            else
                measuredTime =  duration_cast<microseconds>(steady_clock::now() - t0).count();
            return measuredTime;}
    };

    void getColor(float xNorm,  float & r, float & g, float & b);

    struct ind3 {
        int x;
        int y;
        int z;

        ind3(int x, int y, int z) : x(x), y(y), z(z) {}

        bool operator==(ind3 ind3_) {
            return ind3_.x == x and ind3_.y == y and ind3_.z == z;
        }
    };

    struct ind4{
        int x;
        int y;
        int z;
        int w;
        ind4(int x, int y, int z,int w) : x(x), y(y), z(z), w(w) {}
        ind4 () : x(0),y(0),z(0),w(0) {} ;
        int& operator()(int d){
            switch (d) {
                case 0:
                    return x;
                case 1:
                    return y;
                case 2:
                    return z;
                case 3:
                    return w;
                default:
                    cerr << "query on ind4 out of range [0,4)" << endl;
                    return x;
            }
        }
        ind4(ind3 xyz, int w) : x(xyz.x), y(xyz.y), z(xyz.z), w(w){};

    };

    // Point
    struct Point {
        float x;
        float y;
        float z;
        Point(){x = 0; y = 0; z = 0;};
        Point(float x, float y, float z):x(x),y(y),z(z) {};
        Point(const Eigen::Vector3f & pnt):x(pnt(0)),y(pnt(1)),z(pnt(2)) {};
        Point(const geometry_msgs::Point& pnt):x(pnt.x),y(pnt.y),z(pnt.z) {};
        geometry_msgs::Point toGeometry() {
            geometry_msgs::Point pnt;
            pnt.x = x;
            pnt.y = y;
            pnt.z = z;
            return pnt;
        }
        Eigen::Vector3f toEigen() const {
            return Eigen::Vector3f(x,y,z);
        }
        Eigen::Vector3d toEigend() const {

            return Eigen::Vector3d(x,y,z);
        }
        Point operator+(const Point& pnt) const{
            return Point(pnt.x+x,pnt.y+y,pnt.z+z);
        }

        Point operator-(const Point& pnt) const {
            return Point(-pnt.x+x,-pnt.y+y,-pnt.z+z);
        }

        Point operator*(float s) const {
            return Point(s * x, s * y, s * z);
        }

        Point operator/(float s)  const{
            return *this * (1.0/s);
        }

        float distTo(Point p) const {
            return sqrt(pow(p.x-x,2) + pow(p.y-y,2)  + pow(p.z-z,2) ) ;
        }
        float dot(Point p) const{
            return x*p.x + y*p.y + z*p.z;
        }
        float norm() const {return sqrt(pow(x,2) + pow(y,2)  + pow(z,2) );}
        void normalize () { float mag = norm(); x/= mag; y /= mag;  z/= mag; }

        /**
         * |a/|a| - b/|b||
         * @param pnt
         * @return
         */
        float sphereDist(Point pnt) const {
            pnt.normalize();
            Point thisPnt = *this; thisPnt.normalize();
            return pnt.distTo(thisPnt);
        }
        pcl::PointXYZ toPCL() {
            pcl::PointXYZ xyz;
            xyz.x = x;
            xyz.y = y;
            xyz.z = z;
            return xyz;
        }

        /**
         * Get tf rotation to head vantage point from this point
         * @param vantange
         * @return tf rotation matrix
         */
        tf::Matrix3x3 alignX(const Point& vantange){
            tf::Matrix3x3 rot;
            tf::Vector3 e1(vantange.x  - x,vantange.y-y,vantange.z -z); e1.normalize();
            tf::Vector3 e2;
            e2.setX (e1.y()/sqrt( pow(e1.x(),2) + pow(e1.y(),2) ));
            e2.setY (-e1.x()/sqrt( pow(e1.x(),2) + pow(e1.y(),2) ));
            e2.setZ(0);

            tf::Vector3 e3 = e1.cross(e2);
            if (e3.z() < 0 ){
                e2 = -e2;
                e3 = e1.cross(e2);
            }

            rot.setValue(e1.x(),e2.x(),e3.x(),e1.y(),e2.y(),e3.y(),e1.z(),e2.z(),e3.z());
            return rot;
        }

        float angleTo(Point pnt){
            float dot = pnt.x*x + pnt.y*y + pnt.z*z;
            return abs(dot/pnt.norm()/norm());
        }

    };

    Point operator*(float scalar,const Point& pnt );

    // Triangle
    struct Triangle {
        Point vertices[3];
        bool isSOA; // is this mesh source of ambiguity
    };

    // Pose
    struct Pose{
        Eigen::Transform<float,3,Eigen::Affine> poseMat;

        void inverse() {poseMat = poseMat.inverse();}
        Pose() {poseMat.setIdentity();}
        void setTranslation(float x,float y,float z) {poseMat.translate(Eigen::Vector3f(x,y,z));};
        void setTranslation(Point pnt) {poseMat.translate(pnt.toEigen());};
        void setRotation(Eigen::Quaternionf quat) {poseMat.rotate(quat);};

        /**
         * Initialize pose at location x-axis heading to target
         * @param location pose origin
         * @param target observation target
         */
        Pose(Point location, Point target){
            poseMat.setIdentity();
            poseMat.translate(location.toEigen());
            tf::Matrix3x3 rot = location.alignX(target); tf::Quaternion quat;
            rot.getRotation(quat); Eigen::Quaternionf quatEigen(quat.w(),quat.x(),quat.y(),quat.z());
            poseMat.rotate(quatEigen);
        };
        Pose (const geometry_msgs::PoseStamped poseStamped){

            poseMat.setIdentity();
            Eigen::Vector3f loc(poseStamped.pose.position.x,
                                poseStamped.pose.position.y,
                                poseStamped.pose.position.z);

            poseMat.translate(loc);
            Eigen::Quaternionf quaternionf;
            quaternionf.setIdentity();

            quaternionf.w() = poseStamped.pose.orientation.w;
            quaternionf.x() = poseStamped.pose.orientation.x;
            quaternionf.y() = poseStamped.pose.orientation.y;
            quaternionf.z() = poseStamped.pose.orientation.z;

            poseMat.rotate(quaternionf);
        }

        Pose (const tf::StampedTransform& tfStamped){
            poseMat.setIdentity();
            Eigen::Vector3f loc(tfStamped.getOrigin().x(),tfStamped.getOrigin().y(),tfStamped.getOrigin().z());
            poseMat.translate(loc);
            Eigen::Quaternionf quaternionf;
            quaternionf.setIdentity();

            quaternionf.w() = tfStamped.getRotation().w();
            quaternionf.x() = tfStamped.getRotation().x();
            quaternionf.y() = tfStamped.getRotation().y();
            quaternionf.z() = tfStamped.getRotation().z();

            poseMat.rotate(quaternionf);
        }


        tf::StampedTransform toTf(string worldFrameName, string frameName,ros::Time time) const {
            tf::StampedTransform stampedTransform;
            stampedTransform.frame_id_ = worldFrameName;
            stampedTransform.child_frame_id_ = frameName;
            stampedTransform.stamp_ = time;
            stampedTransform.setIdentity();

            tf::Vector3 vec(poseMat.translation().x(),poseMat.translation().y(),poseMat.translation().z());
            stampedTransform.setOrigin(vec);

            Eigen::Quaternionf quatt(poseMat.rotation());
            tf::Quaternion quat(quatt.x(),quatt.y(),quatt.z(),quatt.w());
            stampedTransform.setRotation(quat);
            return stampedTransform;
        }


        Point getTranslation() const {
            Point pnt;
            pnt.x = poseMat.translation().x();
            pnt.y = poseMat.translation().y();
            pnt.z = poseMat.translation().z();
            return pnt;

        }
        /**
         * (x,y,z,w)
         * @return
         */
        Eigen::Quaternionf getQuaternion(){

            return Eigen::Quaternionf(poseMat.rotation());

        }



        void rotate(Eigen::Vector3f axis,float angle){
            poseMat.rotate(Eigen::AngleAxisf(angle,axis));
        }

        void print() const {
            std::cout << poseMat.matrix() <<std::endl;
        }

        geometry_msgs::Pose toGeoPose() const {
            geometry_msgs::Pose pose;

            tf::Vector3 vec(poseMat.translation().x(),poseMat.translation().y(),poseMat.translation().z());
            Eigen::Quaternionf quatt(poseMat.rotation());

            pose.position.x = vec.x();
            pose.position.y = vec.y();
            pose.position.z = vec.z();

            pose.orientation.x = quatt.x();
            pose.orientation.y = quatt.y();
            pose.orientation.z = quatt.z();
            pose.orientation.w = quatt.w();

            return pose;
        }

        void applyTransform (const Pose& pose){
            poseMat = pose.poseMat * poseMat;
        }

    };

    // initialization for poseset
    enum PoseSetInitMethod {
        SPHERE, // spherical initialization
        GRID // grid initialization
        };

    struct PoseSetInitConfig{
        float axisMin[DIM];
        float axisMax[DIM];
        unsigned int step[DIM];
        float delta(int dim ) { return (axisMax[dim] - axisMin[dim])/(step[dim]-1) ;}
        float val(int dim, int idx) {
            if (step[dim] > 1 )
                return axisMin[dim] +delta(dim)*idx;
            else
                return axisMin[dim];
        }
    };

    struct PoseSet{
        public:
        std::vector<Pose> poseSet;
        std::string name = "PoseSet";
        PoseSet() = default;
        PoseSet(int N) {poseSet = std::vector<Pose>(N);}
        PoseSet(std::vector<Point> vantageChain, PoseSetInitMethod initMethod, PoseSetInitConfig initParam){

            int nPose = vantageChain.size() * initParam.step[0] * initParam.step[1] * initParam.step[2];
            poseSet.resize(nPose);

            unsigned int poseCnt = 0 ;
            for (auto targetPnt: vantageChain){
                for (int d1 = 0 ; d1 < initParam.step[0] ; d1++)
                    for (int d2 = 0 ; d2 < initParam.step[1] ; d2++)
                        for (int d3 = 0 ; d3 < initParam.step[2] ; d3++){
                            if (initMethod == PoseSetInitMethod::SPHERE){
                                // axis 1,2,3 = radius, azim, elev
                                Point dir;
                                dir.x = initParam.val(0,d1)*cos(initParam.val(2,d3)) * cos(initParam.val(1,d2));
                                dir.y = initParam.val(0,d1)*cos(initParam.val(2,d3)) * sin(initParam.val(1,d2));
                                dir.z = initParam.val(0,d1)*sin(initParam.val(2,d3));
                                Point camLoc = targetPnt + dir;
                                printf("cam loc: [%f, %f %f]\n",camLoc.x,camLoc.y,camLoc.z);
                                poseSet[poseCnt++] = Pose(camLoc,targetPnt);


                            }else if (initMethod == PoseSetInitMethod::GRID){ //TODO
                                // axis 1,2,3 = x y z


                            }else{
                                // nothing..

                            }

                        }
            }



        }
        void push_back(const Pose& pose){poseSet.emplace_back(pose);}
        void set(unsigned int idx,const Pose& pose ) {poseSet[idx] = pose; }
        Pose get(unsigned int idx) const {return poseSet[idx];}
        unsigned int size() const {return poseSet.size();}
    };

    struct PoseTraj{
        vector<Pose> poses;
        vector<float> ts;
    };


    struct PointSet{
        vector<Point> points;
        int size() const {return points.size();};
        float bearingFrom(Point pnt);
        pcl::PointCloud<pcl::PointXYZ> toPCL(string frame_id);
        Point center() ;
        void push_back(const Point& pnt) {points.push_back(pnt);}
        void push_back(const PointSet& pntSet) {points.insert(points.end(),pntSet.points.begin(),pntSet.points.end());}
    };


    struct LineSegment{
        Point p1;
        Point p2;
        LineSegment() = default;
        LineSegment(Point p0_,Point p1_):p1(p0_),p2(p1_){};
        float distTo(LineSegment l) const ;
        float length() const {return p1.distTo(p2);};
        PointSet samplePoints(float ds,bool includeEnds = false) const;
        PointSet samplePoints(unsigned int nPoints) const;

    };

    struct Path{
        vector<Point> points;
        Path() = default;
        Path(const vector<Point> points_){points = points_;}
        virtual void append(const Point & pnt) {points.push_back(pnt);}
        visualization_msgs::Marker get_point_marker(std_msgs::ColorRGBA color,string frame_id,int id = 0 );
        visualization_msgs::Marker getLineStrip(std_msgs::ColorRGBA color, float width ,string frame_id, string ns, int id =0);
        pcl::PointCloud<pcl::PointXYZ> get_point_pcl(string frame_id);
        pcl::PointCloud<pcl::PointXYZI> get_point_pcl_timecolor(string frame_id,int nStartColor = 0);
        nav_msgs::Path toNavPath(string frame_id, int strideStep = 1);
        int get_num_points() {return points.size();}
        visualization_msgs::MarkerArray get_bearing(visualization_msgs::Marker arrowBase, Path targetPath,int id = 0);
        PointSet toPointSet() {PointSet pntSet; pntSet.points = points; return pntSet;}
        void applyTransform(Pose T01); //! transform all points w.r.t frome {1}
    };

    struct PolynomialXYZ;

    struct Traj : public Path{

        Traj() = default;
        Traj(Path path,vector<float> ts) : ts(ts) {
            points = path.points;
        }
        Traj(vector<Point> pnts, vector<float> ts) : ts(ts){
            points = pnts;
        }
        vector<float> ts; //! time knots
        void append(const Point& pnt,float t) {points.push_back(pnt); ts.push_back(t);}
        Point eval (float t) const;
        float get_length (float t0,float tf) const ;
        int getSize() const {return points.size();}
        float diff(const Traj& traj,float weightMinMaxRatio = 1); //! traj difference with the other
        float diff(const PolynomialXYZ& poly, float weightMinMaxRatio = 1); //! traj difference with the other
    };



    struct TrajStamped{
    public:
        Traj traj; // ts here is local time (e.g. start from zero)
        ros::Time refTime;
        TrajStamped() = default;;
        TrajStamped(ros::Time refTime, Traj traj):
                refTime(refTime),traj(traj) {};
        Point eval(ros::Time evalTime) const {
            double tEval = (evalTime - refTime).toSec();
            return traj.eval(tEval);
        };
    };




    float interpolate(const vector<float>& xData,const vector<float>& yData, const float& x,bool extrapolate);

    struct GridParam{
        Point origin;
        float resolution;
        float lx;
        float ly;
        float lz;
        float min_z = -1e+5;
        GridParam() = default;
    };

    struct BooleanGridParam : public GridParam{
        int nTraj; //! number of inspected instances
        BooleanGridParam(int nTraj, double resolution,const PointSet& targets,
                         double margin_xy,double margin_z,double minZ,double maxZ,
                         bool & isValidField);
    };

    typedef pair<ind3,vector<int>> gridTrajPair; // cell (i,j,k) - list of trajectory index, in the association
    typedef pair<int,vector<ind3>> trajGridPair; // traj index - list of cell (i,j,k)  in the association

    class TraverseGrid{
    private:
        string myName = "[CheckGrid]";
        BooleanGridParam param;
        bool isFieldInit = false;
        bool ****data = NULL; //! [i,j,k,n] where n = check instance index
        int Nx,Ny,Nz,Nt;

        bool isInRange(int ind_,int dim) const ;
        bool isInRange(ind3 ind) const ;
        bool isInRange(Point pnt) const ;

        vector<gridTrajPair> gridTrajPairSet;
        vector<trajGridPair> trajGridPairSet;

        void setTraverse(const Point& pnt, int trajIdx, bool isTraverse = true);
        void printStr(string msg) const {cout << myName << " " << msg << endl;}

        int nTrue = 0; //! number of elements whose value is true

    public:

        ind4 ind2sub(int linIdx) const;
        int sub2ind(ind4 subIdx) const ;
        ind3 pnt2ind(Point pnt) const ;
        Point ind2pnt(ind3 ind) const;
        bool get(ind3 ind, int trajIdx) const {return data[ind.x][ind.y][ind.z][trajIdx];}
        void set(ind3 ind, int trajIdx, bool val) {data[ind.x][ind.y][ind.z][trajIdx] = val;}
        void set(ind4 ind, bool val) { data[ind.x][ind.y][ind.z][ind.w] = val; }
        ind3 getMaxAxis() const  {return ind3(Nx,Ny,Nz);}

        TraverseGrid(BooleanGridParam param);

        void computeGridTrajPairSet();
        void computeTrajGridPairSet();
        int markTraversed(const PointSet& pointSet, int trajIdx, bool & isAllPointsInGrid); //! mark traversed cells by pointSet

        pcl::PointCloud<pcl::PointXYZI> visualizeAt(int trajIdx,string worldFrameId,bool onlyTraverse = false) const; // for trajIdx, visualize traversed cells
        pcl::PointCloud<pcl::PointXYZI> visualizeAll(string frameId) const; // visualize traversed cells by all trajectory
        visualization_msgs::Marker visualizeAllMarker(string frameId,
                                                      std_msgs::ColorRGBA colorAssociation,
                                                      std_msgs::ColorRGBA colorNonAssociation,
                                                      string ns="") const; // visualize traversed cells by all trajectory

        int getNumAssociation() const {return nTrue;}
        void setNumAssociation(int n) {nTrue = n;}
        vector<gridTrajPair> getGridTrajPairSet() const {return gridTrajPairSet;}
        vector<trajGridPair> getTrajGridPairSet() const {return trajGridPairSet;}

    };


    /**
    class ScoreGridBase{
    private:
        GridParam gridParam;
    public:

        bool isFieldInit = false;
        int Nx,Ny,Nz;
        float *** data = NULL;

        ind3 pnt2ind(Point pnt) const ;
        Point ind2pnt(ind3 ind) const;

        bool isInRange(int ind_,int dim) const ;
        bool isInRange(ind3 ind) const ;
        bool isInRange(Point pnt) const ;

        ind3 getMaxAxis() {return ind3(Nx,Ny,Nz);}
        void setValue(ind3 ind, float val) {data[ind.x][ind.y][ind.z] = val;}
        ScoreGridBase(){};
        ScoreGridBase(GridParam param);
    };


    class VisibilityScoreGrid : public ScoreGridBase{

        int dirBorderX[12] = {-1,1,1,-1,0,0,0,0,-1,1,1,-1};
        int dirBorderY[12] = {-1,-1,1,1,-1,1,1,-1,0,0,0,0};
        int dirBorderZ[12] = {0,0,0,0,-1,-1,1,1,-1,-1,1,1};

        // 3d traverse direction
        int dirSectionX[8] = {-1,-1, 1,1, 1,1, -1,-1 };
        int dirSectionY[8] = {-1,-1, -1,-1, 1,1, 1,1 };
        int dirSectionZ[8] = {-1,1, -1,1, -1,1, -1,1 };

        int sweepingDirX[20] = {-1,1,1,-1,0,0,0,0,-1,1,1,-1,   -1,-1, 1,1, 1,1, -1,-1 };
        int sweepingDirY[20] = {-1,-1,1,1,-1,1,1,-1,0,0,0,0,   -1,-1, -1,-1, 1,1, 1,1 };
        int sweepingDirZ[20] = {0,0,0,0,-1,-1,1,1,-1,-1,1,1,   -1,1, -1,1, -1,1, -1,1 };

        struct State{
            Point lastQueriedTargetPoint;
            bool isEDT = false;
            double elapseMem = -1;
            double elapseComp = -1;
            vector<int> notMe;
        };

    private:
        State state;
        GridParam scoreparam;
        void traverse(ind3 targetInd, int startDirIdx , int endDirIdx,bool allowBorder = false);
        ind3 n_go(ind3 dir);

    public:
        VisibilityScoreGrid() {};
        VisibilityScoreGrid(GridParam param):
                ScoreGridBase(param),scoreparam(param){
        };
        void init(GridParam param);
        bool compute(Point target);
        ~VisibilityScoreGrid(){ cout << "Destructing vsf ... " << endl;  };
        void report();
    };
    **/


    /**
     * structure for DAG solving
     */
    struct DirectedGraph{

        /**
         * logging of graph construction
         */
        struct Report{
            vector<int> nFeasibleNodes; //! if nFeasible.size() < N, aborted due to no viable layer
            vector<int> nRejectEdgesTargetCollision; //!
            vector<int> nRejectEdgesDistanceAllowable;
            vector<int> nRejectEdgesTraverseObstacleCollision;
            vector<int> nEdgesFromPrevious;
            vector<int> nFeasibleAndConnectedNodes;

            double elapseConstruction;
            double elapseSolve;
            void init() ;
        };
        Report log;
        Eigen::MatrixXf nodes; //! column = [t n x y z] (n : cam idx in layer)
        Eigen::MatrixXf edges; //! column = [t u v w] (u,v : node idx )
        Eigen::MatrixXf optimal_cost_to_nodes; //! column = [optimal_cost_to_the_node / optimal_parent]
        int* edge_div_location; //! edge_div_location[t] : the first col where the edge from time t-1 to time (t) appears for the first time
        int* node_div_location; //! node_div_location[t] : the first col where the node of time t appears for the first time
        int N_node = 0; //! number of valid node
        int N_edge = 0;
        int N = 0; //! number of tried layers
        Eigen::VectorXi optimal_node_idx_seq;
        bool isSolved = false;
        DirectedGraph() = default;
        pcl::PointCloud<pcl::PointXYZI> getPCLPath(string frame_id); //! visualize only nodes which are connected to at lease one nodes in the previous layer
        void report();
        bool edge_relax();
        bool solve();
    };


    void copyPntAt(sensor_msgs::PointCloud2& pcl, int n , float x, float y, float z);
    vector<vector<int>> product(const std::vector<std::vector<int>>& lists);
    vector<int> hist(const vector<float>& data,Eigen::VectorXf edges,float * var = NULL);
    std::string to_string_with_precision(const float a_value, const int n = 2);


    struct PolynomialDivOutput;

    double B(uint n,uint d);


    double polyEval(const double* polyCoeff,int order,double x);

    class Polynomial{
    private:
        double* polyCoeff;/**< coeff[i] = p_i ( i = 0,..,N)*/
        unsigned int order; /**< polynomial order */
        bool isShallowCopied = false;

    public:
        bool isInit() {return not(polyCoeff==nullptr);};
        Polynomial() {
            order = 0 ; polyCoeff = nullptr;} ;
        Polynomial(double* polyCoeffIn,int polyOrderIn,bool isShallow = false);
        Polynomial(const VectorXd& coeffVec);
        Polynomial(const Polynomial& rhs);
        Polynomial(const size_t& order);

        // Interfacing with raw buffers
        void setData( double* newPolyCoeff,size_t order_);
        void copyData(double* newPolyCoeff, size_t order_);
        void setZero();
        double* getData() const {return polyCoeff;};
        unsigned int getOrder() const {return order;};
        VectorXd getVector() const { return Map<Matrix<double,-1,1>>(polyCoeff,order+1,1);}

        double coeff (unsigned int index) const;
        double& coeffRef (unsigned int index);

        Polynomial& operator=(const Polynomial& rhs);

        Polynomial operator+(const Polynomial& rhs) const;
        Polynomial operator-(const Polynomial& rhs) const;
        Polynomial operator*(const double& rhs) const; /**< scalar product */
        friend Polynomial operator*(double scalar,const Polynomial& rhs);
        Polynomial operator*(const Polynomial& rhs) const; /**< polynomial product using FFT */


        Polynomial& operator*=(double c);
        Polynomial& operator+=(const Polynomial& rhs);
        Polynomial& operator-=(const Polynomial& rhs);
        Polynomial& operator+=(double c);
        Polynomial& operator-=(double c);

        PolynomialDivOutput div(const Polynomial& rhs) const;
        size_t nRoot2(double start,double end) const;

        Polynomial derviative() const;
        double integration(double x1, double x2) const;
        double eval(double x) const;
        double eval(double x,uint dOrder) const;
        ~Polynomial();
    };

    struct PolynomialDivOutput{
        Polynomial q; // quotient
        Polynomial r; // Remainder
        PolynomialDivOutput(const Polynomial& q,const Polynomial& r):q(q),r(r){};
    };

    struct PolynomialXYZ{
        Polynomial px;
        Polynomial py;
        Polynomial pz;
        bool isInit() {return (px.isInit() and px.isInit() and  pz.isInit()) ;};
        PolynomialXYZ(){};
        PolynomialXYZ(std::size_t order):px(order),py(order),pz(order){};
        Polynomial operator [] (int i) const ;
        Polynomial& coeffRef (int i) ;

        double coeff (int i,int j ) const;
        uint getOrder(int i ) const ;
        void setX(const Polynomial& px_);
        void setY(const Polynomial& py_);
        void setZ(const Polynomial& pz_);


        PolynomialXYZ(Polynomial* pxPtr,Polynomial* pyPtr,Polynomial* pzPtr);
        Vector3f evalVec(double t) const;
        Vector3f evalVec(double t,uint dOrder) const;
        Point evalPnt (double t) const;
        PointSet evalPntSet(VectorXd ts) const;
        Point evalPnt (double t,uint dOrder) const ;

        Traj toTraj(float t0,float tf, float dt) const;
        Traj toTraj(float t0,float tf, int N) const;

        void print() const;

    };


    /**
     * @brief compute the polynomial d = (p-c)'A(p-c) where p,c is polynomial and A is PD sym matrix
     * @param outPoly
     * @param px
     * @param py
     * @param pz
     * @param c
     * @param A
     */
    inline void computeRiemannianDistance2(Polynomial &outPoly, const PolynomialXYZ &pxyz,
                                           const PolynomialXYZ &c,
                                           const Matrix3d &A) {
        // We will assume deg(x) >= deg(c) and the outPoly = 0

        uint orderSet[3]; // order of p-c per dimension
        uint orderSetP[3]; // order of p
        uint orderMax = 0;
        for (uint i = 0 ; i < 3 ; i++) {
            orderSetP[i] = pxyz.getOrder(i);
            orderSet[i] = max(pxyz.getOrder(i),c.getOrder(i));
            if (orderMax < orderSet[i])
                orderMax = orderSet[i];
        }

        // construct p-c
        double* pc[3];
        for (uint i =0 ; i < 3 ; i++) {
            pc[i] = new double[orderSet[i]+1];
            for (uint j = 0 ; j <= orderSetP[i] ; j++) // fill p
                pc[i][j] = pxyz.coeff(i,j);
            for (uint j = 0 ; j <= c.getOrder(i) ; j++) // fill -c
                pc[i][j] -= c.coeff(i,j);
        }

        double d[2*orderMax+1]; // d = (p-c)'A(p-c)
        for (uint i = 0 ; i <= 2*orderMax ; i++)
            d[i] = 0;

        // 1. Diagonal terms
        for (uint i = 0 ; i < 3 ; i++) { // per dim
            for (uint j = 0; j <= orderSet[i]; j++)
                d[2 * j] += A(i, i) * pow(pc[i][j], 2);
            for (uint j = 0; j <= orderSet[i]; j++)
                for (uint k = j +1 ; k<=orderSet[i];k++)
                    d[j+k] += 2*A(i, i) * pc[i][j]*pc[i][k];

        }
        // 2. Off diagonal terms
        for (uint i = 0 ; i < 3 ; i++) // per dim
            for (uint j = i+1 ; j < 3;j++)
                if (A(i,j)!=0)
                    for (uint n1 = 0 ; n1 <= orderSet[i] ; n1++) // per order
                        for (uint n2 = 0 ; n2 <= orderSet[j] ; n2++)
                            d[n1+n2]+=2*A(i,j)*pc[i][n1]*pc[j][n2];

        outPoly.copyData(d,2*orderMax);
    }


    struct Permutator
    {
    public:
        Permutator(int s, int v);
        ~Permutator(){delete a;}
        bool doNext();
        int* getOutputArr();

    private:
        int *a;
        int cSlots;
        int cValues;
        int nextInd;
        int* outputArr;
    };


    struct PolynomialHelper{
        PolynomialHelper(int order) : order(order) {};
        int order; //! polynomial order

        MatrixXd scaleMat(double dt);
        VectorXd tVec(double t, uint d);
        double B(uint n,uint d);
        MatrixXd intDerSquared(uint d,double T);
        MatrixXd intDerSquared(uint d,double T,int order);

    };

    nav_msgs::Odometry poseToOdom(const geometry_msgs::PoseStamped& poseStamped);
    visualization_msgs::Marker getClearingMarker(string worldFrame, string ns, int id);

    struct ObjectState{
        ros::Time stamp;
        Point velocity;
        Point acceleration;
        Point position;
        float yaw = 0.0;
        geometry_msgs::Pose toGeoPose(){
            chasing_utils::Pose pose;
            pose.setTranslation(position);
            pose.setRotation(Eigen::Quaternionf(AngleAxisf(yaw,Vector3f::UnitZ())));
            return pose.toGeoPose();
        }
    };

    struct EllipsoidNoRot{
    private:
        Eigen::Matrix3d A;
        Eigen::Vector3d u;
        Eigen::Vector3d c;

    public:
        EllipsoidNoRot() {A = Eigen::Matrix3d::Zero() ; u = Eigen::Vector3d::Zero() ; c = Eigen::Vector3d::Zero();}
        EllipsoidNoRot(Eigen::Vector3d c, Eigen::Vector3d u):c(c),u(u){
            A.setZero();
            for (int d = 0 ; d < 3 ; d++){
                A.coeffRef(d,d) = 1/pow(u(d),2);
            }
        }

        double evalDist (Point pnt) const {
            Eigen::Vector3d pnt_(pnt.x,pnt.y,pnt.z);
            return max(((pnt_-c).transpose()*A*(pnt_-c)).coeff(0)-1,0.0);
        }

    };


    /**
     * Ellipsoidal obstacle
     */
    struct EllipsoidObstacle{
    private:
        PolynomialXYZ r; /**< trajectory of the center (time variant)*/
        Matrix3f A; /**< A = Ru^2R' s.t (x-r)'A(x-r) = 1 */
        Vector3f u; /**< semi-length of each principal axis */
        Pose R; /**< Rwe or R01 */
    public:
        double tLastUpdate; // last filter update. eval of poly = (tEval - tLast)
        static int obstacleIndex;
        EllipsoidObstacle(const PolynomialXYZ& r_,const Vector3f&  u_,const Pose R_):u(u_),R(R_){
            Matrix3f U = Matrix3f::Zero(3,3);
            for (uint d = 0; d < DIM ; d++) {

                U.coeffRef(d,d) = 1/u_(d);
            }
            r = PolynomialXYZ(r_.px.getOrder());
            r.px = r_.px;
            r.py = r_.py;
            r.pz = r_.pz;
            A = R.poseMat.rotation().matrix()*U*U*(R.poseMat.rotation().matrix().transpose());
        }
        Matrix3f getA() const {return A;};
        PolynomialXYZ getr() const  {return r;};
        visualization_msgs::MarkerArray  getMarkerTrace(double t0,double tf,string frame_id,int & nsId,
                                                        double rValue = 0,double gValue = 0 , double bValue = 0,double aValue = 0.7,double scale = 1.0);
        ~EllipsoidObstacle(){//obstacleIndex--;
            //
        };

    };

    int sign(float x);
    float bearingAngle(PointSet targets,Point observer);
    bool collisionRay (octomap_server::EdtOctomapServer* edf_ptr,
                       Point pnt1, Point pnt2,float stride ,float eps = 0);

}


#endif
