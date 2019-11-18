# auto_chaser

<img src="https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/logo.png" width="400"> 
<img src="https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/introl_final.png"> 
<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/algo_explain.gif">
</p>

*This package is devoted to generate an online chasing trajectory for Mavs for videographic tasks.
Chaser is assumed to be provided with either 1)future trajectory of target during a short horizon (info mode) or 2) sparse waypoints for filming (predict mode).*  

**Youtube link for details**  
[auto chaser(info mode)](https://youtu.be/-2d3uDlYR_M ) (IROS2019 accepted/[paper](https://arxiv.org/pdf/1904.03421.pdf))
  
[auto chaser + prediction for target (predict mode)](https://youtu.be/_JSwXBwYRl8) (ICRA2020 submitted/[paper]())

# Overview

## 1 Introduction 





# 2 Installation 

We recommend to use this package in **ros-kinectic** (Ubuntu 16.04). 

## 2.1 Dependencies 

### traj_gen (with qpoases)

The package is trajectory generation library which is used for smooth path generation. 

[download here](https://github.com/icsl-Jeon/traj_gen)



### chomp_predict 

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
sudo apt-get install ros-kinetic-qt-build
```



# 3.  Usage

## 3.0 Common procedure - map and target trajectory 

The chasing algorithm receives 1) prior map (in the form of octomap) and 2) The future trajectory of target in either fully known([one shot mode](#oneshot)) or partially updated way. The environments provided are still to be updated more. Sorry for my laziness ...  

*1) Map*

 In "worlds" folder, you can find *.bt files (e.g. map3.bt which appears in the figures of this page).  You can build your own map by following the [instruction](https://github.com/ethz-asl/rotors_simulator/wiki/Generate-an-octomap-from-your-world) of rotors simulator wiki 

*2) target trajectory*

 In "data/${map name}" folder, you can find path*.txt files which can be loaded in the gui of the package. You may also generate your own path by traj_gen package. For  saving and loading a target path, [this page](https://github.com/icsl-Jeon/traj_gen) is referred(traj_gen).     
<a name="without"></a>
##  3.1 Simulation without gazebo 

This mode does not require gazebo. It only tests the proposed chasing policy and display the result in Rviz.   Please run the following command:

```
roslaunch auto_chaser simulation_without_gazebo.launch
```

### Step 1 : Spawning chaser from user selection 

This step should precede the following steps. In rviz tool properties widget, set the topic name of 2D Nav Goal as /chaser_init_pose. If you don't remember the topic, just click the ```set chaser pose button``` in gui in the next time :). If things done, move on to one of the following two modes : one-shot or  receding horizon method  

### Step 2 - option A :  One-shot mode (offline trajectory computation) 

<img src= "https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/tutorial1.gif">

For this one-shot mode, the chaser is assumed to have the full information for the future trajectory of target. Based on the future path, the trajectory will  be computed immediately.



### Step 2 - option B : Receding horizon mode (online trajectory computation) 

<img src= "https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/tutorial2.gif">

For the receding horizon method, the chaser does not have full information of the future movement of target. Instead, the future movement of target for only short horizon (**"pred_horizon"** parameter in launch files) with a regular interval (**pred_horizon** - **early_end_time** . Please check it in launch files!) is fed to chaser and chaser should keep re-plan in response to the updates.  

## 3.2 Simulation with gazebo

This package provides a simulation environment by employing  package "rotors_simulator". There, the chaser is equipped with vision-sensor (vi-sensor) where you can check the actual capture of the target. In contrast to the [*without gazebo*  mode](#without), the initial spawn pose of the chaser   should be set with the arguments **chaser_x** , **chaser_y**.  

``` 
roslaunch auto_chaser simulation_with_gazebo.launch
```

The remaining procedure is the same with the section 3.1.   

#### In case of real target (More explanation should be added)
User should modify ```/opt/ros/kinetic/share/turtlebot_gazebo/launch/includes/kobuki.launch.xml```
in order to add additional arguments for convenience. 

```
  <arg name="base"/>
  <arg name="stacks"/>
  <arg name="3d_sensor"/>
  <arg name="init_pose"/>
  <arg name="robot_name"/>
  
  <arg name="urdf_file" default="$(find xacro)/xacro.py '$(find turtlebot_description)/robots/$(arg base)_$(arg stacks)_$(arg 3d_sensor).urdf.xacro'" />
  <param name="robot_description" command="$(arg urdf_file)" />
  
  <!-- Gazebo model spawner -->
  <node name="spawn_turtlebot_model" pkg="gazebo_ros" type="spawn_model"
        args=" -unpause -urdf $(arg init_pose) -param robot_description -model $(robot_name)"/>
  
```



# 4. ROS Node API 

## 4.1  Published and subscribed topics

## 4.2 Parameters  

## 
