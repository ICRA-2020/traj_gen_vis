#include "auto_chaser/ObjectHandler.h"

ObjectsHandler::ObjectsHandler(ros::NodeHandle nh){
    // frames
    nh.param<string>("world_frame_id",this->world_frame_id,"/world");
    nh.param<string>("target_frame_id",this->target_frame_id,"/target__base_footprint");
    nh.param<string>("chaser_frame_id",this->chaser_frame_id,"/firefly/base_link");            
    nh.param("min_z",min_z,0.4);            

    target_pose.header.frame_id = world_frame_id;
    chaser_pose.header.frame_id = world_frame_id;
    
    // topics 

    // octomap            
    nh.param("is_octomap_full",this->is_octomap_full,true);
    octree_ptr.reset(new octomap::OcTree(0.1)); // arbitrary init
    if (is_octomap_full)
        sub_octomap = nh.subscribe("/octomap_full",1,&ObjectsHandler::octomap_callback,this);   
    else
        sub_octomap = nh.subscribe("/octomap_binary",1,&ObjectsHandler::octomap_callback,this);   

    ROS_INFO("Object handler initialized.");                         
};

void ObjectsHandler::octomap_callback(const octomap_msgs::Octomap& msg){
    // we receive only once from octoamp server
    if (not is_map_recieved){
        octomap::AbstractOcTree* octree;

        if(is_octomap_full)
            octree=octomap_msgs::fullMsgToMap(msg);
        else
            octree=octomap_msgs::binaryMsgToMap(msg);

        this->octree_ptr.reset((dynamic_cast<octomap::OcTree*>(octree)));
        ROS_INFO_ONCE("[Handler] octomap received.");
        is_map_recieved = true;
    }
};

// retrive 
PoseStamped ObjectsHandler::get_target_pose() {
    PoseStamped pose(target_pose); 
    pose.pose.position.z = min_z; 
    return pose;
};
 
PoseStamped ObjectsHandler::get_chaser_pose() {return chaser_pose;};

octomap::OcTree* ObjectsHandler::get_octree_obj_ptr() {return octree_ptr.get();};

// callback 
void tf_update(){
    for (int i=0;i<2;i++){            
        tf::StampedTransform transform;    
        // 
        try{
            tf_listener.lookupTransform(world_frame_id,objects_frame_id[i],ros::Time(0), transform);
            PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = world_frame_id;

            pose_stamped.pose.position.x = transform.getOrigin().getX();
            pose_stamped.pose.position.y = transform.getOrigin().getY();
            pose_stamped.pose.position.z = transform.getOrigin().getZ();

            pose_stamped.pose.orientation.x = transform.getRotation().getX();
            pose_stamped.pose.orientation.y = transform.getRotation().getY();
            pose_stamped.pose.orientation.z = transform.getRotation().getZ();
            pose_stamped.pose.orientation.w = transform.getRotation().getW();

            objects_pose[i] = pose_stamped;
            is_env_ok[i] = true;

            if (i==0)
                {ROS_INFO_ONCE("tf of target received. "); is_target = true;} 
            else
                {ROS_INFO_ONCE("tf of chaser received. "); is_chaser = true;}  

        }
        catch (tf::TransformException ex){
            if (i==0)
                ROS_ERROR_ONCE("tf of target does not exist. ",ex.what());  
            else
                ROS_ERROR_ONCE("tf of chaser does not exist. ",ex.what());  
        
        }
    }


}