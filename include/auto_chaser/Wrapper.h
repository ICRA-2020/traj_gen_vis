// auto chasing wrawpper 

#include "auto_chaser/Chaser.h"
#include "auto_chaser/ObjectHandler.h"

class Wrapper{
    private:

        ros::Publisher pub_control_mav; // publish the control pose for UAV (trajectory_msgs)        
        ros::Publisher pub_control_mav_vis;  // publish the control pose visualization(geometry_msgs)
        ros::Publisher pub_chaser_control_path; // history of pose desired of chaser (not actual history)
        ros::Publisher pub_connecting_vel_marker; // for debugging purpose : is continuity forced properlly?
        ros::Publisher pub_target_future_segment; // target future segment which was utilized for chasing trigger

        geometry_msgs::PoseStamped control_pose_mav; 
        nav_msgs::Path path_control_pose; // flight history
        

        // for debugging purposes (continuity of each trajectory segment)    
        visualization_msgs::Marker velocity_marker_base; 
        visualization_msgs::MarkerArray velocity_marker_array; 

    public:    
        
        int run_mode; // without gazebo 
        ObjectsHandler objects_handler;
        Chaser chaser;
        nav_msgs::Path target_future_seg; // target path during a short horizon (visualization purpose)

        Wrapper();
        void init(ros::NodeHandle nh);
        void session(double t); // publish things 
        
        bool trigger_chasing(TimeSeries chasing_knots); // no information given from target manager 
        bool trigger_chasing(vector<Point> target_seq,TimeSeries chasing_knots); //information given from target manager         
        geometry_msgs::PoseStamped get_control_pose(double t_eval); // get the latest control pose 
        
        void pub_control_pose(double t_eval); 
        void pub_control_traj(double t_eval);        
        void pub_control_pose(PoseStamped control_pose); 
        void pub_control_traj(PoseStamped control_pose);
        void pub_control_path(double t_eval);
        void pub_connecting_velocity_marker();
        void pub_target_segment();


        void write(string log_path); // log in txt file
};