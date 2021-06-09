//
// Created by jbs on 21. 6. 7..
//
#include <chasing_utils/TargetManager.h>


int main(int argc,char** argv) {

    ros::init(argc,argv,"target_manager");
    octomap_server::EdtOctomapServer edtServer;
    chasing_utils::TargetManager targetManager(&edtServer);
//    chasing_utils::TargetManager targetManager;
    vector<chasing_utils::PredictionOutput> predictionOutput;

    while(ros::ok()){
        if (targetManager.needPrediction())
            if (not targetManager.predict(predictionOutput))
                ROS_ERROR("prediction failed");

        ros::spinOnce();
        ros::Rate(30).sleep();
    }

    return 0;
}
