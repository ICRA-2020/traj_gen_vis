#ifndef OBJ_HANDLE_H
#define OBJ_HANDLE_H
#include "auto_chaser/Common.h"
class ObjectsHandler{
    private:
        // ros
        ros::Subscriber sub_octomap;
        ros::Publisher pub_edf;
        visualization_msgs::Marker markers_edf; // Euclidean distance field   
        tf::TransformListener tf_listener; // don't have initial copy constructor 
        
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
        FieldParams edf_grid_params;

        // parameter 
        double min_z; // the minimum height to be clamped  
        double edf_max_viz_dist;
        double edf_max_dist;

    public:
        //flag
        bool is_octomap_full = false;
        bool is_chaser_recieved = false;
        bool is_map_recieved = false;        
        bool is_target_recieved = false;
        

        ObjectsHandler(){};
        void init(ros::NodeHandle nh);
        ObjectsHandler(ros::NodeHandle nh);
        PoseStamped get_target_pose(); 
        PoseStamped get_chaser_pose(); 
        octomap::OcTree* get_octree_obj_ptr(); 

        void octomap_callback(const octomap_msgs::Octomap& msg);
        void tf_update();
        void compute_edf();

};





#endif
