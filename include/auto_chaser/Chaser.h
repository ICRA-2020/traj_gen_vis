#include "auto_chaser/Preplanner.h"
#include "auto_chaser/SmoothPlanner.h"


class Chaser{
    private:
        Preplanner preplanner;
        SmoothPlanner smooth_planner;

        ros::Publisher pub_control_mav;
        geometry_msgs::PoseStamped pose_control_mav;        
        
        // subroutines 
        void preplan(); 
        void path_complete();       


    public:
        bool is_complete_chasing_path; // is there any complete chasing path

        Chaser();
        void init(ros::NodeHandle nh);
        bool chase_update(GridField* global_edf,vector<Point> target_pnts,Point chaser_x0,Twist chaser_v0,Twist chaser_a0,TimeSeries knots); // load control point 
        void session(double t); // publish information 
        
        // evaluation with current 
        Point eval_point(double t_eval);
        Twist eval_velocity(double t_eval);
        Twist eval_acceleration(double t_eval);
        void publish_control(double t_eval);
        Point get_control_point(double t_eval);

};


