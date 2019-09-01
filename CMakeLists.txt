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
qt_build trajectory_msgs  mav_msgs chomp_predict
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

##############################################################################
# Qt Environment
##############################################################################
set(CMAKE_AUTOUIC ON) 
# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in manifest.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here
set(QT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/qt_ui_infomode)
set(QT_FORMS ${QT_DIR}/mainwindow.ui) # by adding this, the catkin build make ui_mainWindow.h
file(GLOB QT_MOC   ${QT_DIR}/*.h)
file(GLOB QT_SOURCES ${QT_DIR}/*.cpp)
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
message("qt form: ${QT_DIR}")
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
message("qt form path: ${CMAKE_CURRENT_BINARY_DIR}")


## prior information mode 
add_executable(${PROJECT_NAME}_run src/qt_ui_infomode/main.cpp ${AUTO_CHASER_LIBS} ${QT_FORMS_CPP} ${QT_SOURCES} ${QT_MOC_HPP} ${qpOASES_LIBS} )
target_link_libraries(${PROJECT_NAME}_run ${catkin_LIBRARIES} ${QT_LIBRARIES})


## prediction mode 
add_executable(${PROJECT_NAME}_run_predict_mode src/qt_ui_predictmode/main_test.cpp ${AUTO_CHASER_LIBS}  ${qpOASES_LIBS} )
target_link_libraries(${PROJECT_NAME}_run_predict_mode ${catkin_LIBRARIES})


## anlyzer 
add_executable(${PROJECT_NAME}_analyzer src/analyzer/anal_main.cpp ${AUTO_CHASER_LIBS}  ${qpOASES_LIBS} src/analyzer/anal_server.cpp )
target_link_libraries( ${PROJECT_NAME}_analyzer ${catkin_LIBRARIES})



