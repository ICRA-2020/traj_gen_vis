cmake_minimum_required(VERSION 2.8.3)
project(auto_chaser)

add_compile_options(-std=c++11)
#set(CMAKE_BUILD_TYPE Debug)
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

#qpOASES

set(qp_SRC_DIR /home/jbs/lib/qpOASES-3.2.1/src)
message("path : ${qp_SRC_DIR}")
file(GLOB qpOASES_LIBS ${qp_SRC_DIR}/*.cpp)
message("path : ${qpOASES_LIBS}")

file(GLOB AUTO_CHASER_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/src/auto_chaser/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/target_manager/*.cpp)
message("auto chaser libs : ${AUTO_CHASER_LIBS}")


# ROS 
find_package(catkin REQUIRED roscpp 
sensor_msgs tf geometry_msgs octomap_msgs message_generation nav_msgs traj_gen gazebo_msgs
trajectory_msgs  mav_msgs chomp_predict
)


message("catkin path: ${catkin_INCLUDE_DIRS}")

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES auto_chaser
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)


## prediction mode 
add_executable(${PROJECT_NAME}_run_predict_mode src/qt_ui_predictmode/main_test.cpp ${AUTO_CHASER_LIBS}  ${qpOASES_LIBS} )
target_link_libraries(${PROJECT_NAME}_run_predict_mode ${catkin_LIBRARIES})


## anlyzer 
add_executable(${PROJECT_NAME}_analyzer src/analyzer/anal_main.cpp ${AUTO_CHASER_LIBS}  ${qpOASES_LIBS} src/analyzer/anal_server.cpp )
target_link_libraries( ${PROJECT_NAME}_analyzer ${catkin_LIBRARIES})



