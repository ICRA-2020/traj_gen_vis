#include "auto_chaser/Common.h"
#include "traj_gen/PolyTrajGen.h"

class SmoothPlanner{
    public:
        // objects
        PathPlanner planner;
        TrajGenOpts option;

        // ros 
        ros::Publisher pub_path; // publihser for path 
        ros::Publisher pub_chasing_corridor; 
        string world_frame_id; 
        visualization_msgs::Marker chasing_corridor;
        nav_msgs::Path chasing_smooth_path;

        SmoothPlanner();
        void init(ros::NodeHandle nh);
        void traj_gen(TimeSeries knots,nav_msgs::Path waypoints,Twist v0,Twist a0);
        void publish();
};