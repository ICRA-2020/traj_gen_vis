#include "auto_chaser/SmoothPlanner.h"

SmoothPlanner::SmoothPlanner(){}

void SmoothPlanner::init(ros::NodeHandle nh){

    // paramter parsing for option 
    nh.param<string>("world_frame_id",world_frame_id,"/world");
    nh.param<double>("chaser/safe_corridor_r",option.safe_r,0.5);
    nh.param<int>("chaser/N_safe_pnts",option.N_safe_pnts,3);
    nh.param<int>("chaser/objective_derivative",option.objective_derivative,3);
    nh.param<bool>("chaser/is_multi_corridor",option.is_multi_corridor,true);

    if(option.is_multi_corridor)
        option.is_single_corridor = false;
    else
        option.is_single_corridor = true;
            
    nh.param<bool>("chaser/is_waypoint_soft",option.is_waypoint_soft,true);
    nh.param<double>("chaser/w_deviation",option.w_d,0.5);
    nh.param<int>("chaser/poly_order",option.poly_order,6);
    nh.param<bool>("chaser/verbose",option.verbose,true);

    pub_chasing_corridor = nh.advertise<visualization_msgs::Marker>("chasing_corridor",1); 
    pub_chasing_corridor_single = nh.advertise<visualization_msgs::MarkerArray>("chasing_corridor_single",1); 
    pub_path = nh.advertise<nav_msgs::Path>("smooth_path",1);
    pub_knots = nh.advertise<visualization_msgs::Marker>("smooth_path_knots",1);

    chasing_corridor.header.frame_id = world_frame_id;
    chasing_smooth_path.header.frame_id = world_frame_id;

};


void SmoothPlanner::traj_gen(TimeSeries knots,nav_msgs::Path waypoints,Twist v0,Twist a0){

        
        int i = 0;
        for (auto it = waypoints.poses.begin() ; it < waypoints.poses.end(); it ++,i++){
            cout << "knots " << i <<" :" << it->pose.position.x << " , " << it->pose.position.y << endl; 

        }
        

    /**
     cout << "time knots: " << endl;
     cout << knots << endl;
    **/
    
    planner.path_gen(knots,waypoints,v0,a0,option);
    
    if (planner.is_spline_valid()){
        // markers update 
        chasing_corridor = planner.get_safe_corridor_marker();
        chasing_corridor.header.frame_id = world_frame_id;
        chasing_corridor_single = planner.get_safe_corridor_single_marker();
        chasing_corridor.header.frame_id  = world_frame_id;
        
        chasing_smooth_path = planner.get_path();
        chasing_smooth_path.header.frame_id = world_frame_id;        
        marker_knots = planner.get_knots_marker();
        marker_knots.header.frame_id = world_frame_id;
    }
}

void SmoothPlanner::publish(){
    pub_path.publish(chasing_smooth_path);
    pub_chasing_corridor.publish(chasing_corridor);    
    pub_chasing_corridor_single.publish(chasing_corridor_single);
    pub_knots.publish(marker_knots);
}