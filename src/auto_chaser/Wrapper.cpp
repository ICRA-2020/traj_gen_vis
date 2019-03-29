#include "auto_chaser/Wrapper.h"

Wrapper::Wrapper(){};

void Wrapper::init(ros::NodeHandle nh) {
    objects_handler.init(nh);    
};


