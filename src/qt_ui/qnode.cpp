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

    // target manager init
    this->target_manager.init(nh);

    // wrapper init
    this->chaser_wrapper.init(nh);

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
    writeOnBoard("waiting EDF computation...");

    while(ros::ok()){
        double sim_time;
        if(is_in_session)
            sim_time = previous_elapsed +(ros::Time::now() - button_click_time).toSec();
        else
            sim_time = previous_elapsed; 
        
        // target manager 
        target_manager.session(sim_time);
        chaser_wrapper.session();
        
        // chaser 
        if(chaser_wrapper.objects_handler.is_map_recieved and (not is_said_edf)){
            writeOnBoard("EDF loaded.");
            is_said_edf = true;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }


    Q_EMIT rosShutdown();



}