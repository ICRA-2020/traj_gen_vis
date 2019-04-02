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
void Wrapper::trigger_chasing(){
    
    vector<Point>  target_pred_seq = objects_handler.get_prediction_seq();
    Point chaser_current_point = objects_handler.get_chaser_pose().pose.position;
    GridField * edf_grid_ptr = objects_handler.get_edf_grid_ptr();

    // chasing policy update 
    chaser.chase_update(edf_grid_ptr,target_pred_seq,chaser_current_point);    
}
// one shot 
void Wrapper::trigger_chasing(vector<Point> target_pred_seq){


    Point chaser_current_point = objects_handler.get_chaser_pose().pose.position;
    GridField * edf_grid_ptr = objects_handler.get_edf_grid_ptr();
    // chasing policy update 
    chaser.chase_update(edf_grid_ptr,target_pred_seq,chaser_current_point);    
}



