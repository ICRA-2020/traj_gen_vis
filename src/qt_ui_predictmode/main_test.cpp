#include <auto_chaser/Common.h>
#include <auto_chaser/Wrapper.h>
#include <target_manager/TargetManager.h>


double pred_horizon;
double early_end_time;
int pred_seq;




/**
 * @brief should be triggered only prediction exists 
 * t_cur and t_ros are redundant actually
 * @param predictor 
 * @param chasing_wrapper 
 * @param t_cur simulation time. for numerical stability, this time scale is preferred. This time will be used for chaser
 * @param t_ros ros time. This will be used in chomp prediction. 
 */
bool trigger(TargetPredictor* predictor,Wrapper* chasing_wrapper,double t_cur,ros::Time t_ros){


    // extract the prediction points (ros time)
    VectorXd pred_horizon_vec_ros(pred_seq);
    
    double dt = double(pred_horizon) / pred_seq;

    pred_horizon_vec_ros.setLinSpaced(pred_seq,t_ros.toSec()+dt,t_ros.toSec() + pred_horizon); 
    vector<Point> target_seq = predictor->eval_time_seq(pred_horizon_vec_ros); 

    // trigger chasing (sim time) 
    VectorXd pred_horizon_vec(pred_seq+1);
    pred_horizon_vec.setLinSpaced(pred_seq+1,t_cur, t_cur + pred_horizon); 
    return chasing_wrapper->trigger_chasing(target_seq,pred_horizon_vec);

};

// This code mimic QNode in infomode
int main(int argc, char * argv[]){
    ros::init(argc,argv,"auto_chaser_prediction");
    ros::NodeHandle nh("~");
        
    nh.param("pred_horizon",pred_horizon,(3.0));
    nh.param("early_end_time",early_end_time,0.1);
    nh.param<int>("pred_seq",pred_seq,4);

    TargetPredictor target_predictor;
    Wrapper chaser_wrapper;

    target_predictor.init();
    chaser_wrapper.init(nh);

    bool trigger_chasing_cond = false;
    bool trigger_predict_cond = true;

    ros::Rate loop_rate (20);
    ros::Time t_ros_start = ros::Time::now();
     
    double t_cur;
    double last_chasing_trigger_time = 0 ;
    bool chasing_trigger_condition;
    bool trigger_success = true;

    while(ros::ok()){
        t_cur = (ros::Time::now() - t_ros_start).toSec(); // current simulation time 

        // predictor session 
        // chaser session 
        chaser_wrapper.session(t_cur); // ref time = double (t_sim) 
        bool is_new_prediction = target_predictor.session(); 
        if (is_new_prediction)
            ROS_INFO_STREAM("[DEBUG]: new prediction triggered");
        
        // check the trigger condition 
        chasing_trigger_condition = last_chasing_trigger_time == 0  // If this the first, then predict 
            // or ((t_cur - last_chasing_trigger_time) > pred_horizon - early_end_time)  // if horizon time expired, then trigger 
            or is_new_prediction; // if prediction is triggered newly, then trigger 

        chasing_trigger_condition = chasing_trigger_condition 
                                    and target_predictor.get_forecaster_ptr()->get_predict_condition();
        
        // if conditions are met, trigger chasing trajectory
        if(chasing_trigger_condition){            
            ROS_INFO("[MAIN_TESTER] time : %f - chasing triggered!",t_cur);
            trigger_success = trigger(&target_predictor,&chaser_wrapper,t_cur,ros::Time::now());
            if (not trigger_success)
                ROS_WARN("[MAIN TESTER] current chasing is not reliable.");
            last_chasing_trigger_time = t_cur;
        }
        // logging
        if(chaser_wrapper.objects_handler.is_log){
            // cout << "trigger: " << chaser_wrapper.chaser.is_complete_chasing_path <<endl;

            if (chaser_wrapper.chaser.is_complete_chasing_path){
                string log_dir = chaser_wrapper.objects_handler.log_dir;
                chaser_wrapper.write(log_dir); // wrapper history save
                target_predictor.get_forecaster_ptr()->write(log_dir); // error history 
            }
        }


        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;

}
