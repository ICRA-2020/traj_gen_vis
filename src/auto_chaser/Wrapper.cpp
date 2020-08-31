#include "auto_chaser/Wrapper.h"

Wrapper::Wrapper(){};

void Wrapper::init(ros::NodeHandle nh) {
    
    // the initialization for its own member variables 
    string mav_name;
    nh.param<string>("mav_name",mav_name,"firefly");
    nh.param("run_mode",run_mode,0);

    // this topic is used for the actual control 
    string control_topic = "/" + mav_name + "/" + mav_msgs::default_topics::COMMAND_TRAJECTORY;

    pub_control_mav = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
           control_topic.c_str(), 10);
    pub_chaser_control_path = nh.advertise<nav_msgs::Path>("mav_desired_pose_history",1);        
    // only for visualization 
    pub_control_mav_vis = nh.advertise<geometry_msgs::PoseStamped>("mav_pose_desired",1);
    pub_connecting_vel_marker = nh.advertise<visualization_msgs::MarkerArray>("connecting_vel",1);
    pub_target_future_segment = nh.advertise<nav_msgs::Path>("target_future_segment",1); 
    
    // sub classes initialization 
    objects_handler.init(nh);  
    chaser.init(nh);    

    velocity_marker_base.header.frame_id = objects_handler.get_world_frame_id();
    velocity_marker_base.type = visualization_msgs::Marker::ARROW;
    velocity_marker_base.scale.x = 0.08;
    velocity_marker_base.scale.y = 0.1;
    velocity_marker_base.scale.z = 0.15;
    velocity_marker_base.pose.orientation.w = 1.0;
    velocity_marker_base.color.a =0.2;
    velocity_marker_base.color.r =1.0;
    
    target_future_seg.header.frame_id = objects_handler.get_world_frame_id();

};


void Wrapper::session(double t){
    // (1) the information of objects handler 
    objects_handler.publish();
    objects_handler.tf_update();    
    // (2) chaser
    chaser.session(t);
    // (3) wrapper 
    if (run_mode == 0 ){
        pub_control_pose(t);
        pub_control_traj(t);
        pub_control_path(t);
        pub_connecting_velocity_marker();
        
    }
    else // run mode 1 
        if(chaser.is_complete_chasing_path){        
            ROS_INFO_ONCE("[Wrapper] current chaser pose = planning ");
            pub_control_pose(t);
            pub_control_traj(t);
            pub_control_path(t);
            pub_connecting_velocity_marker();
        }
        else // if no chasing path 
            if (objects_handler.is_chaser_recieved){
                // if chaser pose received    
                ROS_INFO_ONCE("[Wrapper] current chaser pose = desired ");
                
                // pub_control_pose(objects_handler.get_chaser_pose());
                // pub_control_traj(objects_handler.get_chaser_pose());
       
                pub_control_pose(t);
                pub_control_traj(t);

            }else{
                ROS_WARN_ONCE("[Wrapper] chaser pose not recieved. not publising control pose yet");
            }
        
        // publish target future segment
        pub_target_segment();
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
        
        // if there is no previous chasing trajectory, then set the spwan  as initial condition. 
        // This makes sense in case of gazebo          
        if (run_mode == 0){
                        
            if (not objects_handler.is_chaser_spawned){
                // if there is no previous chasing trajectory, then set the current position as initial condition. 
                // This makes sense in case of gazebo
                chaser_init_point.x = chaser.spawn_x;
                chaser_init_point.y = chaser.spawn_y;
                chaser_init_point.z = chaser.hovering_z;
                chaser_init_vel = objects_handler.get_chaser_velocity();
                chaser_init_acc = objects_handler.get_chaser_acceleration();
            }
            else{
                // This is gui version. set initial point as the initial pose of chaser which was selected by user
                chaser_init_point = objects_handler.get_chaser_pose().pose.position;    
                chaser_init_vel = objects_handler.get_chaser_velocity();
                chaser_init_acc = objects_handler.get_chaser_acceleration();   
            }     
            
        }else{                        
            chaser_init_point = objects_handler.get_chaser_pose().pose.position;    
            chaser_init_vel = objects_handler.get_chaser_velocity();
            chaser_init_acc = objects_handler.get_chaser_acceleration();   
        }
            
    }
    else{
        chaser_init_point = chaser.eval_point(t_planning_start);
        chaser_init_vel = chaser.eval_velocity(t_planning_start);
        chaser_init_acc = chaser.eval_acceleration(t_planning_start);

        // in this case, we push back arrow for visualizing the connecting velocity 
        velocity_marker_base.points.clear();
        velocity_marker_base.points.push_back(chaser_init_point);
        double length = 0.4;
        double vel_norm = sqrt(pow(chaser_init_vel.linear.x,2) + 
                pow(chaser_init_vel.linear.y,2) +
                pow(chaser_init_vel.linear.z,2));
        geometry_msgs::Point end_pnt; // end point at the velocity vector 
        end_pnt.x = chaser_init_point.x + chaser_init_vel.linear.x/vel_norm*length;
        end_pnt.y = chaser_init_point.y + chaser_init_vel.linear.y/vel_norm*length;
        end_pnt.z = chaser_init_point.z + chaser_init_vel.linear.z/vel_norm*length;        
        velocity_marker_base.points.push_back(end_pnt);

        velocity_marker_array.markers.push_back(velocity_marker_base);
    }

    // chasing policy update 
    bool is_success = chaser.chase_update(edf_grid_ptr,target_pred_seq,chaser_init_point,chaser_init_vel,chaser_init_acc,chasing_knots);   
    if(is_success) 
        objects_handler.is_path_solved = true; // at least once solved, 

    return is_success;
}

// informative mode 
bool Wrapper::trigger_chasing(vector<Point> target_pred_seq,TimeSeries chasing_knots){
  


    double t_planning_start = chasing_knots(0);

    GridField * edf_grid_ptr = objects_handler.get_edf_grid_ptr();

    Point chaser_init_point;
    Twist chaser_init_vel; 
    Twist chaser_init_acc;

    if(not chaser.is_complete_chasing_path){    
        
        if (run_mode == 0 ){
            
            if (not objects_handler.is_chaser_spawned){
                // if there is no previous chasing trajectory, then set the current position as initial condition. 
                // This makes sense in case of gazebo
                chaser_init_point.x = chaser.spawn_x;
                chaser_init_point.y = chaser.spawn_y;
                chaser_init_point.z = chaser.hovering_z;
                chaser_init_vel = objects_handler.get_chaser_velocity();
                chaser_init_acc = objects_handler.get_chaser_acceleration();
            }
            else{
                // This is gui version. set initial point as the initial pose of chaser which was selected by user
                chaser_init_point = objects_handler.get_chaser_pose().pose.position;    
                chaser_init_vel = objects_handler.get_chaser_velocity();
                chaser_init_acc = objects_handler.get_chaser_acceleration();   
            }                                

        }else{                        
            chaser_init_point = objects_handler.get_chaser_pose().pose.position;    
            chaser_init_vel = objects_handler.get_chaser_velocity();
            chaser_init_acc = objects_handler.get_chaser_acceleration();   
        }
    }
    else{
        // if chasing trajectory was planned, 
        chaser_init_point = chaser.eval_point(t_planning_start);
        chaser_init_vel = chaser.eval_velocity(t_planning_start);
        chaser_init_acc = chaser.eval_acceleration(t_planning_start);   
        

        // in this case, we push back arrow for visualizing the connecting velocity 
        velocity_marker_base.points.clear();
        velocity_marker_base.points.push_back(chaser_init_point);
        velocity_marker_base.id = velocity_marker_array.markers.size();

        double length = 0.4;
        double vel_norm = sqrt(pow(chaser_init_vel.linear.x,2) + 
                pow(chaser_init_vel.linear.y,2) +
                pow(chaser_init_vel.linear.z,2));
        geometry_msgs::Point end_pnt; // end point at the velocity vector 
        end_pnt.x = chaser_init_point.x + chaser_init_vel.linear.x/vel_norm*length;
        end_pnt.y = chaser_init_point.y + chaser_init_vel.linear.y/vel_norm*length;
        end_pnt.z = chaser_init_point.z + chaser_init_vel.linear.z/vel_norm*length;        
        velocity_marker_base.points.push_back(end_pnt);

        velocity_marker_array.markers.push_back(velocity_marker_base);
    }

    // chasing policy update 
    bool is_success = chaser.chase_update(edf_grid_ptr,target_pred_seq,chaser_init_point,chaser_init_vel,chaser_init_acc,chasing_knots);   

    if(is_success) 
        objects_handler.is_path_solved = true; // at least once solved, 

    // chasing policy update 
    return is_success;
}

/**
 * @brief evalate the latest control pose from chaser 
 * 
 * @param t_eval the evaluation time 
 * @return geometry_msgs::PoseStamped the control pose for MAV 
 */
geometry_msgs::PoseStamped Wrapper::get_control_pose(double t_eval){

    PoseStamped target_pose = objects_handler.get_target_pose(); // the current target pose of the latest
    PoseStamped chaser_pose = objects_handler.get_chaser_pose(); // the current chaser pose of the latest (if run_mode = 0 , equal to desired pose)

    if (this->objects_handler.is_log and chaser.is_complete_chasing_path){
        // file write for target 
        std::ofstream wnpt_file;
	// cout <<"[DEBUG] current logging dir = " << objects_handler.log_dir << endl;

        wnpt_file.open((this->objects_handler.log_dir+"/target_history.txt").c_str(),ios_base::app);

        if(wnpt_file.is_open()){
            wnpt_file<<target_pose.pose.position.x<<","<<target_pose.pose.position.y<<","<<target_pose.pose.position.z<<"\n";
            wnpt_file.close();    
        }else
            cout<<"logging file for target pose is not opend"<<endl;

        // file write for chaser 
        std::ofstream wnpt_file2;
        wnpt_file2.open((this->objects_handler.log_dir+"/chaser_history.txt").c_str(),ios_base::app);

        if(wnpt_file2.is_open()){
            wnpt_file2<<chaser_pose.pose.position.x<<","<<chaser_pose.pose.position.y<<","<<chaser_pose.pose.position.z<<"\n";
            wnpt_file2.close();    
        }else
            cout<<"logging file for chaser pose is not opend"<<endl;         

    }




    // decide yawing direction so that the local x-axis heads to the target 
    // For this, if the target has not been uploaded yet, then the MAV heads to the last observed target position 
	//ROS_INFO("[DEBUG: Wrapper] recognized chaser pose : [%f , %f]",target_pose.pose.position.x,target_pose.pose.position.x);
    float yaw = atan2(-chaser_pose.pose.position.y+target_pose.pose.position.y,
                        -chaser_pose.pose.position.x+target_pose.pose.position.x);
    tf::Quaternion q;
    if(not chaser.is_complete_chasing_path)
        q.setRPY(0,0,0); // should hovering first 
    else
        q.setRPY(0,0,yaw);
    q.normalize(); // to avoid the numerical error 
    PoseStamped chaser_pose_desired;  
    Point chaser_point_desired = chaser.get_control_point(t_eval); 
    chaser_pose_desired.header.frame_id = objects_handler.get_world_frame_id();
    chaser_pose_desired.pose.position = chaser_point_desired;
    chaser_pose_desired.pose.orientation.x = q.x();
    chaser_pose_desired.pose.orientation.y = q.y();
    chaser_pose_desired.pose.orientation.z = q.z();
    chaser_pose_desired.pose.orientation.w = q.w();
    return chaser_pose_desired;
}


void Wrapper::pub_control_pose(geometry_msgs::PoseStamped pose){
    pose.pose.position.z = chaser.get_hovering_z(); // why? 
    pub_control_mav_vis.publish(pose);
}


void Wrapper::pub_control_traj(geometry_msgs::PoseStamped chaser_pose_desired){

    // convert the pose information into trajectory_msgs (only conversion)
    chaser_pose_desired.pose.position.z = chaser.get_hovering_z();

    // get the position 
    Vector3d chaser_point_desired;
    chaser_point_desired(0) = chaser_pose_desired.pose.position.x;
    chaser_point_desired(1) = chaser_pose_desired.pose.position.y;
    chaser_point_desired(2) = chaser_pose_desired.pose.position.z;
    // get the orientation 
    tf::Quaternion q;
    q.setX(chaser_pose_desired.pose.orientation.x);
    q.setY(chaser_pose_desired.pose.orientation.y);
    q.setZ(chaser_pose_desired.pose.orientation.z);
    q.setW(chaser_pose_desired.pose.orientation.w);
    tf::Matrix3x3 q_mat(q);
    double roll,pitch,yaw;
    q_mat.getRPY(roll,pitch,yaw);
    // finishing the trajectory topic 
    trajectory_msgs::MultiDOFJointTrajectory chaser_traj_desired;
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(chaser_point_desired,yaw, &chaser_traj_desired);
    pub_control_mav.publish(chaser_traj_desired);
}

/**
 * @brief publish the control visualization (geometry_msgs) 
 * 
 * @param t_eval evaluation time (sim time)
 */
void Wrapper::pub_control_pose(double t_eval){
    pub_control_mav_vis.publish(get_control_pose(t_eval));

}


/**
 * @brief publish control path history (nav_msgs) / Added at 2019/11/11
 * 
 * @param t_eval  evaluation time (sim time)
 */
void Wrapper::pub_control_path(double t_eval){
    path_control_pose.header.frame_id = objects_handler.get_world_frame_id();

    // we append new point to path only if it is different with the previous point than a threshold 
    const double update_thres = 1e-3;

    PoseStamped new_pose_stamped = get_control_pose(t_eval);
    Point new_point = new_pose_stamped.pose.position;

    if (chaser.is_complete_chasing_path){ // we will append after chasing path loaded
        if (path_control_pose.poses.size()){ // of course exception handing in case of first insertion 
            // first, check whether updatable
            Point previous_pose_stamped = path_control_pose.poses.back().pose.position;
            double diff = sqrt(pow(new_point.x - previous_pose_stamped.x,2) + pow(new_point.y - previous_pose_stamped.y,2)); 
            if (diff>update_thres)
                path_control_pose.poses.push_back(new_pose_stamped);             
        }
        else // if this is the first, just push back 
            path_control_pose.poses.push_back(new_pose_stamped);
    }
    
    pub_chaser_control_path.publish(path_control_pose);
}

/**
 * @brief publish control path history (nav_msgs) / Added at 2019/11/11
 * 
 * @param t_eval  evaluation time (sim time)
 */
void Wrapper::pub_connecting_velocity_marker(){
    pub_connecting_vel_marker.publish(velocity_marker_array);
}


/**
 * @brief publish the control visualization (trajectory_msgs) 
 * 
 * @param t_eval 
 */
void Wrapper::pub_control_traj(double t_eval){

    float chaser_yaw_desired;
    PoseStamped chaser_pose_desired;

    // retrieve yaw first  
    chaser_pose_desired = get_control_pose(t_eval);
    
    // convert the pose information into trajectory_msgs (only conversion)
    
    // get the position 
    Vector3d chaser_point_desired;
    chaser_point_desired(0) = chaser_pose_desired.pose.position.x;
    chaser_point_desired(1) = chaser_pose_desired.pose.position.y;
    chaser_point_desired(2) = chaser_pose_desired.pose.position.z;
    // get the orientation 
    tf::Quaternion q;
    q.setX(chaser_pose_desired.pose.orientation.x);
    q.setY(chaser_pose_desired.pose.orientation.y);
    q.setZ(chaser_pose_desired.pose.orientation.z);
    q.setW(chaser_pose_desired.pose.orientation.w);
    tf::Matrix3x3 q_mat(q);
    double roll,pitch,yaw;
    q_mat.getRPY(roll,pitch,yaw);
    // finishing the trajectory topic 
    trajectory_msgs::MultiDOFJointTrajectory chaser_traj_desired;
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(chaser_point_desired,yaw, &chaser_traj_desired);
    pub_control_mav.publish(chaser_traj_desired);
}

/**
 * @brief publish target future segment used in the latest chasing update 
 * 
 */
void Wrapper::pub_target_segment(){
    pub_target_future_segment.publish(target_future_seg);
};

