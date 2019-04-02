// auto chasing wrawpper 

#include "auto_chaser/Chaser.h"
#include "auto_chaser/ObjectHandler.h"

class Wrapper{

    public:    
        ObjectsHandler objects_handler;
        Chaser chaser;
        Wrapper();
        void init(ros::NodeHandle nh);
        void session();   
        bool trigger_chasing(TimeSeries chasing_knots); // no information given from target manager 
        bool trigger_chasing(vector<Point> target_seq,TimeSeries chasing_knots); //information given from target manager 
               
};