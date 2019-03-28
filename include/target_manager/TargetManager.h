#include "traj_gen/PolyTrajGen.h"
#include <tf/transform_broadcaster.h>

class TargetManager{
    private:
        PathPlanner planner;
        string target_frame_id;
        string world_frame_id;
        int mode; // 0(without gazebo), 1(with gazebo), 2(real)
        ros::Publisher pub_target_pose;
        tf::TransformBroadcaster br; 
    public:
        TargetManager(ros::NodeHandle nh) {};
        // used only mode 0 
        void broadcast_target_tf(double);

}