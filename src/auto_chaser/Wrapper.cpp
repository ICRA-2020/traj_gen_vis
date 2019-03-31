#include "auto_chaser/Wrapper.h"

Wrapper::Wrapper(){};

void Wrapper::init(ros::NodeHandle nh) {
    objects_handler.init(nh);    
};


void Wrapper::session(){

    objects_handler.publish();
    objects_handler.tf_update();
}



