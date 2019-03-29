#ifndef COMMON_H
#define COMMON_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Path.h>


#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <eigen3/Eigen/Core>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <string>
#include <memory>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <chrono>

using namespace std;
using namespace Eigen;
using namespace geometry_msgs;

#define GetCurrentDir getcwd

// get the working directory 
std::string GetCurrentWorkingDir( void );


/**
 * Functions 
 */ 

void get_color(float x_in, float & r, float & g, float & b);
void get_color_dist(float dist_val,std_msgs::ColorRGBA& color, float max_plot_dist_val);

/**
 * Structure  
 */

template <typename T>
struct Node{
    T value; // actual value
    string name; // vertex name 
};

// grid field object
struct FieldParams{
    double x0,y0,z0;
    double lx,ly,lz;
    double resolution;
    double max_plot_dist_val; // we plot the voxel within this dist val    
    double ray_stride_res;
};


// field = f(x,y,z) : 3D grid field
struct GridField{
    FieldParams params;    
    VectorXf node_xs,node_ys,node_zs;
    vector<Point> pnts_list;
    int Nx,Ny,Nz;
    vector<Point> saved_points; 
    float*** field_vals;
    GridField(); 
    GridField(FieldParams param); 
    // ~GridField() {delete[] field_vals; std::cout<<"[Field] destructed"<<std::endl; }; // destructor 
    
    // generate a set of nodes from points in the grid field. this will be used as layer in a graph  
    vector<Node<Point>> generate_node(int prefix);
    // reset the origin of the field 
    void setOrigin(Point X0);
    // the origin of entire field 
    Point getOrigin();
    // get the center point of origin voxel
    Point getCentre();    
    Point getCellPnt(Vector3i idx);
    float getValue(Point pnt);
    Vector3i getCellIdx(Point pnt);
    // all index along a line connecting pnt1 and pnt2 
    vector<Vector3i> getRayIdx(Point pnt1,Point pnt2);
    // this is important function: get the minimum value along the line (minimum clamped)
    float getRayMin(Point pnt1,Point pnt2,float clamping_val);
    // this is important function: get the mean value along a line (for weight)
    float getRayMean(Point pnt1,Point pnt2);

    void updateCell(Point pnt,float val);
    void updateCell(Vector3i idx,float val);
    int getNumCell(){return Nx*Ny*Nz;}

};





#endif
