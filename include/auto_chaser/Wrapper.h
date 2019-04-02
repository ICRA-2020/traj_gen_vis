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
        void trigger_chasing(); // no information given from target manager 
        void trigger_chasing(vector<Point> target_seq); //information given from target manager 
               
};