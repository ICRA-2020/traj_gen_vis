#include "auto_chaser/Chaser.h"

Chaser::Chaser(){}

void Chaser::init(ros::NodeHandle nh){

    preplanner.init(nh);
}
void Chaser::chase_update(GridField* global_edf_ptr,vector<Point> target_pnts,Point chaser_init){
    // phase 1 pre planning 
    preplanner.preplan(global_edf_ptr,target_pnts,chaser_init);
    
}

void Chaser::session(){
    preplanner.publish(); // markers     
}