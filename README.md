
<img src="https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/logo.png" width="400"> 
<img src="https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/introl_final.png"> 
<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/algo_explain.gif">
</p>

### Generation of target chasing trajectory under obstacle environment 

*This ros package is devoted to generate an online chasing trajectory for Mavs for videographic tasks.
Chaser is assumed to be provided with either 1)future trajectory of target during a short horizon (info mode) or 2) sparse waypoints for filming (predict mode). This algorithm ensures safety, travel efficiency and visibility having real-timeness and optimality in minde.*  

**Youtube link for details**  
[auto chaser(info mode)](https://youtu.be/-2d3uDlYR_M ) (IROS2019 accepted/[paper](https://arxiv.org/pdf/1904.03421.pdf))
  
[auto chaser + prediction for target (predict mode)](https://youtu.be/_JSwXBwYRl8) (ICRA2020 submitted/[paper](https://arxiv.org/pdf/1911.09280.pdf))

**Tutorials on this are ready soon in youtube**.      

# Getting started

## 1. Installation 

We recommend to use this package in **ros-kinectic** (Ubuntu 16.04). The belows are required package before building this package.

### traj_gen (with qpoases)

The package is trajectory generation library which is used for smooth path generation. 

[download here](https://github.com/icsl-Jeon/traj_gen)

### chomp_predict (use *only_octomap* branch)

The package is prediction module in case of unknown future trajectory  based on Covariant optimization 

[download here](https://github.com/icsl-Jeon/chomp_predict)

### rotors_simulator

The package is gazebo simulator for MAV. This is used for simulation of chasing planner in a virtual MAV platform

[download here](https://github.com/ethz-asl/rotors_simulator)

### octomap

 We use octomap to represent the environment. dynamicEDT3D libraries also should be installed for Euclidean distance transform field(EDF). The visibility score field(VSF) will be computed based on the EDF.    

[download here](http://github.com/OctoMap/octomap)

### Others

```
$ sudo apt-get install ros-kinetic-qt-build ros-kinetic-octomap-server 
$ cd catkin_ws/src
$ git clone https://github.com/andreasBihlmaier/gazebo2rviz.git
$ git clone https://github.com/andreasBihlmaier/pysdf.git
```
### \*Optional 
<a name="optinal"></a>

For the users hoping to try the algorithm by hand-operated mobile robot in *prediction mode(explained below).* 
```
$ sudo apt-get install ros-kinetic-turtlebot-gazebo 
$ sudo apt-get install ros-kinetic-turtlebot-teleop 

```

### Build this package

```
$ cd catkin_Ws/src
$ git clone https://github.com/icsl-Jeon/traj_gen_vis.git
$ catkin build 

```

## 2. Introduction
In this package, we provide two modes for different cases. For both cases, we first assume that the map is provided as ".bt" file. Please check how they are loaded in all the launch files. If the chaser can access the exact future trajectory of moving target over a total duration *(one-shot)* or over a time window *(receding horzion)*, we call it *1) informed mode*. In contrast, we call it *2) prediction mode* if target future motion is unkown except a sequence of waypoints. In prediction_mode, user is recommended to provide prior target waypoints which is ensured to be passed by target one-by-one. In either mode, users can simulate the algorithm with or without gazebo. In cases where the camera view of drone is not necessary and additional comuputing power for running gazebo is not affordable, users can still visualize with *rviz* in *without-gazebo* mode.     


### Informed mode *(info_mode)*
<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/info_overview.gif" width="1024">
</p>

In the informed mode, The trajectory of target is to be prebuilt or loaded by users through gui. Target trajectory is built on waypoints(knots) from a constrained Quadratic programming ([see *traj_gen* for demo](https://github.com/icsl-Jeon/traj_gen)).  
First, just try out the exsiting waypoints for target trajectory (use data/map3/path3.txt to load in gui).
Users can get the total chasing trajectory recieving the entire target trajectory by clicking *one-shot*, or simulate the algorithm in receding horizon manner in either with gazebo or without gazebo. ***Tutorials on this are ready soon in youtube***.      

 
### Prediction mode *(pred_mode)*
<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/pred_overview.gif" width="1024">
</p>

In prediction mode, we use [*chomp_predict*](https://github.com/icsl-Jeon/traj_gen) to properlly forecast the future movement of target considering obstacles. To run *auto chaser* in this mode properrly, user is required to have [target waypoints information](https://github.com/icsl-Jeon/chomp_predict/blob/master/params/chomp_param_map3.yaml) loaded in launch file. You can run algorithm by one the two ways: 

1) The users can provide the bag file which publishes `/target_pose` as observation to be fed into the chaser for the prediction
```
roslaunch auto_chaser pred_simulation_with_gazebo_bag.launch
```

2) The users can also test algorithm with more realistic situation in gazebo by spawning and operating *turtlebot* rather than offline-gathered bag_file. This requires additional [dependencies](#optional) 
```
roslaunch auto_chaser pred_simulation_with_gazebo_keyboard.launch
```



### Launch summary*
<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/github.png" width="1024">
</p>


