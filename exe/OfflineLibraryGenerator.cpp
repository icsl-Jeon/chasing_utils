#include <chasing_utils/Library.h>

using namespace std;
using namespace chasing_utils;

void headerUpdate(visualization_msgs::MarkerArray& markerArray){
    for (auto it = markerArray.markers.begin() ; it != markerArray.markers.end() ; it++)
        it->header.stamp = ros::Time::now();
}

/**
 * This code generate a set of library and save it to binary file
 * Also, the generated trajs and traverseGrid will be visualized in Rviz.
 * Or in the load mode, just read the files and publish markers
 */
int main(int argc, char** argv){

    ros::init(argc,argv,"library_manager");
    ros::NodeHandle nh("~/library");

    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("lib_visualizations",1);
    ros::Publisher pubAssociation = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("lib_visualizations_association",1);
    lib::LibraryParam param(nh);

    /**
     * For SP method, we simply test with a straight x-line
     */
    if (param.strategy == lib::Strategy::SKELETON_PERMUTATION){
        VectorXd px (2); px << 0.5 , 2.0; Polynomial polyX(px);
        VectorXd py (1); py << 0.0; Polynomial polyY(py);
        VectorXd pz (1); pz << 0.0; Polynomial polyZ(pz);
        param.SP_polyRefPtr = new PolynomialXYZ(&polyX,&polyY,&polyZ);
    }

    lib::Library library(param,0);

    if (not library.isInit()){
        ROS_ERROR("library initialization failed. Aborting");
        abort();
    }

    visualization_msgs::MarkerArray markers = library.getMarkerTotal("map");
    pcl::PointCloud<pcl::PointXYZI> points = library.getMarkerAssociation("map");

    while(ros::ok()){
        headerUpdate(markers);
        points.header.stamp = pcl_conversions::toPCL(ros::Time::now());
        pub.publish(markers);
        pubAssociation.publish(points);
        ros::Rate(30).sleep();
    }

    return 0;
}


