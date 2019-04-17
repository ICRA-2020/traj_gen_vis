#include "auto_chaser/Chaser.h"

Chaser::Chaser():is_complete_chasing_path(false){}

void Chaser::init(ros::NodeHandle nh){

    preplanner.init(nh);
    smooth_planner.init(nh);

    // retreieve initial hovering command 
    nh.param("chaser_init_x",spawn_x,0.0);
    nh.param("chaser_init_y",spawn_y,0.0);
    nh.param("chaser_init_z",hovering_z,1.0);
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

void Chaser::session(double t){
    preplanner.publish(); // markers     
    smooth_planner.publish();  // markers 
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


/**
 * @brief obtains the latest control point. yaw will be selected from wrapper with information of target   
 * 
 * @param t_eval evaluation time 
 * @return Point the control point 
 */
Point Chaser::get_control_point(double t_eval){    
    if (this->is_complete_chasing_path)
        return smooth_planner.planner.point_eval_spline(t_eval); 
    else{// hovering command at the spawning position with desired height      
            Point hovering_point;
            hovering_point.x = spawn_x;
            hovering_point.y = spawn_y;
            hovering_point.z = hovering_z;        
        return hovering_point;
    }
}