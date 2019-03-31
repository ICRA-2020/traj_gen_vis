#include "auto_chaser/Common.h"


class Preplanner{

    private:
        chaser::PreplannerParams params;

        vector<shared_ptr<GridField>> vsf_field_ptr_seq; // visibility score field sequence 
        Graph di_graph; // directed graph for preplanning 
        DescriptorMap descriptor_map;

        ros::Publisher pub_vsf_vis; // vsf field pub
        ros::Publisher pub_preplanned_path;  
        ros::Publisher pub_waypoints; 

        visualization_msgs::Marker markers_visibility_field_base;
        visualization_msgs::MarkerArray markers_visibility_field_seq;
        visualization_msgs::Marker marker_wpnts; // waypoints of preplanned path
        nav_msgs::Path preplanned_path; // for visualization and path completion 

        // subroutine functions
        void compute_visibility_field_seq(vector<Point> target_pnts); // local vsf 
        void graph_construct( GridField* global_edf,vector<Point> target_pnts,Point x0);        
        void get_shortest_path();   
        VertexPath dijkstra(Vertex_d v0,Vertex_d vf);
        

    public:
        Preplanner();
        void init(ros::NodeHandle nh);
        void preplan(GridField* global_edf,vector<Point> target_pnts,Point chaser_init);
        void publish();
        nav_msgs::Path get_preplanned_waypoints();
}