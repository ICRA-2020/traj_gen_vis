#include "auto_chaser/Preplanner.h"
#include "auto_chaser/SmoothPlanner.h"


class Chaser{
    private:
        Preplanner preplanner;
        SmoothPlanner smooth_planner;
        
        // subroutines 
        void preplan(); 
        void path_complete();       
        
    public:
        Chaser();
        void init(ros::NodeHandle nh);
        void chase_update(GridField* global_edf,vector<Point> target_pnts,Point chaser_init); // load control point 
        void session(); // publish information 
        
};


