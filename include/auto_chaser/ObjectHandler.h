#ifndef OBJ_HANDLE_H
#define OBJ_HANDLE_H
#include "auto_chaser/Common.h"
class ObjectsHandler{
    private:


        // ros
        ros::Subscriber sub_octomap;
        visualization_msgs::MarkerArray markers_edf;        
        tf::TransformListener tf_listener; // don't have initial copy constructor 
        
        // id         
        string world_frame_id;
        string chaser_frame_id;
        string target_frame_id;
        
        // topic
        string octomap_topic_name;    
        // env (octomap)
        std::shared_ptr<octomap::OcTree> octree_ptr;

        // objects
        PoseStamped target_pose; 
        PoseStamped chaser_pose; 

        
    public:
        //flag
        bool is_octomap_full = false;
        bool is_chaser_recieved = false;
        bool is_map_recieved = false;        
        bool is_target_recieved = false;


        ObjectsHandler(){};
        ObjectsHandler(ros::NodeHandle nh);
        // subscribe and update distmap 
        void octomap_callback(const octomap_msgs::Octomap& msg){
            octomap::AbstractOcTree* octree;

            if(is_octomap_full)
                octree=octomap_msgs::fullMsgToMap(msg);
            else
                octree=octomap_msgs::binaryMsgToMap(msg);

            this->octree_ptr.reset((dynamic_cast<octomap::OcTree*>(octree)));
            ROS_INFO_ONCE("[Handler] octomap received.");
            is_octomap_init = true;
            is_env_ok[2]=true;

        }
        // sanity check
        bool is_ok(){            
            string msg[3] = {"target","chaser","octomap"};
 
            if (sub_octomap.getNumPublishers() == 0)
                is_env_ok[2] =false;            
            else
                is_env_ok[2] =true;


            if ( not (is_env_ok[0] and is_env_ok[1] and is_env_ok[2])){
                ROS_WARN("retriving is not perfect ----------------");
                for (int n = 0 ;n<3;n++)
                    if(not is_env_ok[n])
                        cout<<(msg[n]+string(" is not received"))<<endl;
            } 
            return (is_env_ok[0] and is_env_ok[1] and is_env_ok[2]);
        }
    
};





#endif
