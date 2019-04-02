#include "auto_chaser/Chaser.h"

Chaser::Chaser(){}

void Chaser::init(ros::NodeHandle nh){

    preplanner.init(nh);
    smooth_planner.init(nh);

    
}
bool Chaser::chase_update(GridField* global_edf_ptr,vector<Point> target_pnts,Point chaser_x0,Twist chaser_v0,Twist chaser_a0,TimeSeries knots){
    
    bool result = false;

    // phase 1 pre planning 
    preplanner.preplan(global_edf_ptr,target_pnts,chaser_x0);
    ROS_INFO("[Chaser] preplanning completed.");

    nav_msgs::Path waypoints = preplanner.get_preplanned_waypoints();

    if(waypoints.poses.size()){
        // phase 2 smooth planning     
        smooth_planner.traj_gen(knots,waypoints,chaser_v0,chaser_a0);
        if (smooth_planner.planner.is_spline_valid())
            {ROS_INFO("[Chaser] smooth path completed."); is_complete_chasing_path = true; return true;}
        else 
            ROS_WARN("[Chaser] smooth path incompleted.");
    }else
        ROS_WARN("[Chaser] preplanning failed");

    return false;
}

void Chaser::session(){
    preplanner.publish(); // markers     
    smooth_planner.publish(); 
}


Point Chaser::eval_point(double t_eval){    
    return smooth_planner.planner.point_eval_spline(t_eval);        
}


Twist Chaser::eval_velocity(double t_eval){
    return smooth_planner.planner.vel_eval_spline(t_eval);        
}

Twist Chaser::eval_acceleration(double t_eval){
    return smooth_planner.planner.accel_eval_spline(t_eval);        
}