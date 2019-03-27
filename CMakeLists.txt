cmake_minimum_required(VERSION 2.8.3)
project(auto_chaser)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED)

#OCTOMAP 
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

#EDT 
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

# EIGEN 
find_package(Eigen3 REQUIRED NO_MODULE)

# ROS 
find_package(catkin REQUIRED roscpp sensor_msgs tf geometry_msgs octomap_msgs message_generation nav_msgs traj_gen)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_gen_chasing
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)
