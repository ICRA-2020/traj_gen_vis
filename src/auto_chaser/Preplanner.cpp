#include "auto_chaser/Preplanner.h"

Preplanner::Preplanner(){};

void Preplanner::init(ros::NodeHandle nh){

    // preplanner params parsing 
    nh.param("w_v",params.w_v,0.3);            
    nh.param("r_safe",params.r_safe,2.0);
    nh.param("min_z",params.min_z,0.4);
    nh.param("w_v",params.w_v,0.3);
    nh.param("vs_min",params.vs_min,0.3);
    nh.param("vsf_resolution",params.vsf_resolution,0.5);
    nh.param("d_connect_max",params.d_connect_max,3.0);

    nh.param("d_trakcing_max",params.d_trakcing_max,5.0);
    nh.param("d_trakcing_min",params.d_trakcing_min,0.6);
    nh.param("max_azim",params.max_azim,(3.141592/4));


    // world_frame_id 
    nh.param<string>("world_frame_id",markers_visibility_field_base.header.frame_id,"/world");


    // marker initialize 

    // waypoints 
    marker_wpnts.header.frame_id = markers_visibility_field_base.header.frame_id;
    marker_wpnts.ns = "waypoints";
    marker_wpnts.id = 0;
    marker_wpnts.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_wpnts.color.r = 14.0/255.0;
    marker_wpnts.color.g = 50.0/255.0;
    marker_wpnts.color.b = 1.0;
    marker_wpnts.color.a = 0.8;
    marker_wpnts.pose.orientation.w = 1.0;
    double scale = 0.3;
    marker_wpnts.scale.x = scale;
    marker_wpnts.scale.y = scale;
    marker_wpnts.scale.z = scale;    

    


};