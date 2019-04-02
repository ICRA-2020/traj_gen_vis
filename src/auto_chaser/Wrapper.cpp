#include "auto_chaser/Wrapper.h"

Wrapper::Wrapper(){};

void Wrapper::init(ros::NodeHandle nh) {
    objects_handler.init(nh);  
    chaser.init(nh);      
};


void Wrapper::session(){

    objects_handler.publish();
    objects_handler.tf_update();
    
    chaser.session();
}

// chasing session called 
bool Wrapper::trigger_chasing(TimeSeries chasing_knots){
    

    double t_planning_start = chasing_knots(0);

    vector<Point>  target_pred_seq = objects_handler.get_prediction_seq();
    GridField * edf_grid_ptr = objects_handler.get_edf_grid_ptr();
    

    Point chaser_init_point;
    Twist chaser_init_vel; 
    Twist chaser_init_acc;

    if(not chaser.is_complete_chasing_path){    
        chaser_init_point = objects_handler.get_chaser_pose().pose.position;    
        chaser_init_vel = objects_handler.get_chaser_velocity();
        chaser_init_acc = objects_handler.get_chaser_acceleration();   
    }
    else{
        chaser_init_point = chaser.eval_point(t_planning_start);
        chaser_init_vel = chaser.eval_velocity(t_planning_start);
        chaser_init_acc = chaser.eval_acceleration(t_planning_start);
    }

    // chasing policy update 
    chaser.chase_update(edf_grid_ptr,target_pred_seq,chaser_init_point,chaser_init_vel,chaser_init_acc,chasing_knots);   
}

// one shot from static initial 
bool Wrapper::trigger_chasing(vector<Point> target_pred_seq,TimeSeries chasing_knots){
  

    Point chaser_init_point = objects_handler.get_chaser_pose().pose.position; // current position
    Twist chaser_init_vel = objects_handler.get_chaser_velocity(); // zero
    Twist chaser_init_acc = objects_handler.get_chaser_acceleration(); //zero
    GridField * edf_grid_ptr = objects_handler.get_edf_grid_ptr();
    
    // chasing policy update 
    return chaser.chase_update(edf_grid_ptr,target_pred_seq,chaser_init_point,chaser_init_vel,chaser_init_acc,chasing_knots);   

}



