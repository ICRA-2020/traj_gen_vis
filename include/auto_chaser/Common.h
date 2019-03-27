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

// #include <opencv2/contrib/contrib.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>

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

#endif