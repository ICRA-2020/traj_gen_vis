#include "analyzer/anal_server.h"

using namespace std;
int main(int argc, char* argv[]){
    
    ros::init(argc,argv,"anal_server");
    ros::NodeHandle nh("~");    
    
    std::string octomap_file_name;
    std::string chaser_history_file_name;
    std::string target_history_file_name;        
    std::string write_path;        
    double width;
    double alpha;
    int arrow_draw_step;

    double max_vs;
    
    nh.param<string>("octomap_file",octomap_file_name,"/home/jbs/catkin_ws/src/traj_gen_vis/worlds/map2.bt");
    nh.param<string>("chaser_file",chaser_history_file_name,"/home/jbs/catkin_ws/src/traj_gen_vis/log/chaser_history.txt");
    nh.param<string>("target_file",target_history_file_name,"/home/jbs/catkin_ws/src/traj_gen_vis/log/target_history.txt");
    nh.param<string>("write_path",write_path,"/home/jbs/catkin_ws/src/traj_gen_vis/log");
    nh.param("arrow_thickness",width,0.10);
    nh.param("arrow_alpha",alpha,0.2);
    nh.param("arrow_draw_step",arrow_draw_step,1);
    std::cout<<"arrow draw step in the main loop "<<arrow_draw_step<<std::endl;



    ros::Rate rate(100);
    AnalServer anal(octomap_file_name,chaser_history_file_name,target_history_file_name,width,alpha,arrow_draw_step);
    anal.write_path = write_path;
    anal.pub_result = nh.advertise<visualization_msgs::MarkerArray>("anal_result",1);
    anal.pub_chaser_path = nh.advertise<nav_msgs::Path>("chaser_path",1);
    anal.pub_target_path = nh.advertise<nav_msgs::Path>("path_path",1);
    anal.pub_edf = nh.advertise<visualization_msgs::Marker>("edf",1);
    
    for(int i =0;i<N_point_plot;i++){
        anal.pub_pose[i] = nh.advertise<geometry_msgs::PoseStamped>(("pose_seq"+to_string(i)).c_str(),1);
        anal.chaser_pos_vec[i].header.frame_id = "/world";
    }

    anal.write();
    ros::Rate loop(30);
	ROS_INFO("analyzer publishing");
	while(ros::ok()){
        anal.publish();        
		ros::spinOnce();
		loop.sleep();

    }

    return 0;
}
