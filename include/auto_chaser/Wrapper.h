// auto chasing wrawpper 

#include "auto_chaser/Chaser.h"
#include "auto_chaser/ObjectHandler.h"

class Wrapper{

    public:    
        ObjectsHandler objects_handler;
        Wrapper();
        void init(ros::NodeHandle nh);
        void session();

};