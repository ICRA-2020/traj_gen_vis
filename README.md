
# traj_gen_vis

<img src="https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/introl_final.png"> 

*This package is devoted to generate an online chasing trajectory in response to the future movement of a moving target only for a short horizon. It assumes that the future trajectory of target is updated with a given time interval and priori map is given in the form of Octomap*  

*I also wish that my package can provide a test enviroment for comparing different chasing algorithm*

# 1 Overview 
## 1.1 Algorithm
<img src="https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/overview.png"> 

## 1.2 File structure 
[doxygen](https://icsl-jeon.github.io/)

# 2 Installation 

## 2.1 Dependencies 

### traj_gen (with qpoases)

The package is trajectory generation library which is used for smooth path generation

[download here](https://github.com/icsl-Jeon/traj_gen)


### rotors_simulator 

The package is gazebo simulator for MAV. This is used for simulation of chasing planner in a virtual MAV platform

[download here](https://github.com/ethz-asl/rotors_simulator)

### octomap 

We use octomap to represent the environment. dynamicEDT3D libraries also should be installed for Euclidean distance transform field(EDF). The visibility score field(VSF) will be computed based on the EDF.    

# 3 Usage 


## 3.1 Simulation without gazebo 

### (1) one-shot mode 

### (2) 

## 3.2 Simulation with gazebo (still developing)

# 4. ROS Node API 

## 4.1  Published and subscribed topics


## 4.2 Parameters  
