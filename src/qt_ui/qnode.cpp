#include "qnode.h"
// constructor 
QNode::QNode(int argc, char** argv, const std::string &name ) :
    init_argc(argc),
    init_argv(argv),
    node_name(name)
    {

}
QNode::~QNode(){}
void QNode::ros_comms_init(){
    
    ros::NodeHandle nh("~");

    // taarget manager init
    this->target_manager.init(nh);

}

void QNode::shutdown() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

bool QNode::on_init(){

    ros::init(init_argc,init_argv,node_name);
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
    ros_comms_init();
    start();
    return true;
}

void QNode::run(){
    ros::Rate loop_rate(50);

    while(ros::ok()){
 
        // target manager 
        target_manager.session();
        // chaser 

        ros::spinOnce();
        loop_rate.sleep();
    }


    Q_EMIT rosShutdown();



}