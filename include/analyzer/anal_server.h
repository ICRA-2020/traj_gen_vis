#include "auto_chaser/Wrapper.h"
#include "sensor_msgs/Range.h"
#include "tf/LinearMath/Quaternion.h"
const int N_point_plot = 10;
using namespace std;
class AnalServer{
    public:
        std::string octomap_file_name;
        std::string chaser_history_file_name;
        std::string target_history_file_name;  
        std::string write_path;
        

        vector<Point> chaser_history;
        vector<Point> target_history;   
        nav_msgs::Path chaser_path;
        nav_msgs::Path target_path;
        visualization_msgs::Marker arrow;
        visualization_msgs::MarkerArray arrow_array;
        visualization_msgs::Marker edf_marker;

        vector<geometry_msgs::PoseStamped> chaser_pos_vec;
        vector<sensor_msgs::Range> chaser_range_vec;


        ros::Publisher pub_result;
        ros::Publisher pub_chaser_path;
        ros::Publisher pub_target_path;
        ros::Publisher pub_edf;
        ros::Publisher pub_pose[N_point_plot];
        ros::Publisher pub_sensor[N_point_plot];

        // log 
        vector<float> clutter_target;
        // vector<float> clutter_chaser;
        vector<float> clutter_bearing;
        vector<float> clutter_chaser; 
        vector<float> vel_target;
        float chaser_travel;
        float arrow_width;
        float arrow_alpha;
        int arrow_draw_step;
        AnalServer(std::string octomap,std::string chaser,std::string target,float arrow_width,float arrow_alpha,int arrow_draw_step);
        void analyze();
        void write();
        void publish();

};
