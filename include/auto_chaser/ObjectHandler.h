#ifndef OBJ_HANDLE_H
#define OBJ_HANDLE_H
#include "auto_chaser/Common.h"
class ObjectsHandler{
    private:
        // ros
        ros::Subscriber sub_octomap;
        ros::Subscriber sub_chaser_init_pose; // user input from rviz
        ros::Publisher pub_edf;
        visualization_msgs::Marker markers_edf; // Euclidean distance field   
        tf::TransformListener* tf_listener; // don't have initial copy constructor 
        tf::TransformBroadcaster* tf_talker; // chaser tf broadcast in mode 0

        // id         
        string world_frame_id;
        string chaser_frame_id;
        string target_frame_id;
        

        // topic
        string octomap_topic_name;    

        // objects
        PoseStamped target_pose; 
        PoseStamped chaser_pose; 
        shared_ptr<octomap::OcTree> octree_ptr;
        DynamicEDTOctomap *edf_ptr; 
        shared_ptr<GridField> edf_grid_ptr; // signed distance field

        // parameter 
        double min_z; // the minimum height to be clamped  
        double chaser_init_z; // initial target hovering height 
        double edf_max_viz_dist;
        double edf_max_dist;
        FieldParams edf_grid_params; 
        int run_mode;  // 0 (simulation without gazebo) , 1 (simulation with gazebo)  
        
         
    public:
        //flag
        bool is_octomap_full = false;
        bool is_chaser_recieved = false;
        bool is_map_recieved = false;        
        bool is_target_recieved = false;
        bool is_control_received = false;
        bool is_chaser_spawned = false;
        bool is_insert_permit = false;

        ObjectsHandler(){};
        void init(ros::NodeHandle nh);
        void compute_edf();
        ObjectsHandler(ros::NodeHandle nh);
        PoseStamped get_target_pose(); 
        PoseStamped get_chaser_pose(); 
        octomap::OcTree* get_octree_obj_ptr(); 
        GridField* get_edf_grid_ptr(); 

        void octomap_callback(const octomap_msgs::Octomap& msg);
        void chaser_spawn(PoseStamped spawn_pose);
        void callback_chaser_init_pose(const geometry_msgs::PoseStampedConstPtr& chaser_init_pose);
        void tf_update();
        void publish();
        vector<Point> get_prediction_seq();

};





#endif
