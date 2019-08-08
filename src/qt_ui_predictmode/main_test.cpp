#include <auto_chaser/Common.h>
#include <auto_chaser/Wrapper.h>
#include <target_manager/TargetManager.h>



int main(int argc, char * argv[]){


    ros::init(argc,argv,"auto_chaser_prediction");
    ros::NodeHandle nh("~");

    TargetPredictor target_predictor;
    Wrapper chaser_wrapper;

    target_predictor.init();
    chaser_wrapper.init(nh);

    bool trigger_cond = false;
    ros::Rate loop_rate (20);

    while(ros::ok){
        if(target_predictor.get_forecaster_ptr()->is_predicted)
            ROS_INFO("[Target Predictor] prediction started. procede chasing.");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}