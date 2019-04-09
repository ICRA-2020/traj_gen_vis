
# traj_gen_vis

<img src="https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/introl_final.png"> 

*This package is devoted to generate an online chasing trajectory in response to the future movement of a moving target only for a short horizon. It assumes that the future trajectory of target is updated with a given time interval and priori map is given in the form of Octomap*  

# 1 Overview 
## 1.1 Algorithm
<img src="https://github.com/icsl-Jeon/traj_gen_vis/blob/master/img/overview.png"> 

## 1.2 File structure 
The detailed class and headers can be found here: [doxygen](https://icsl-jeon.github.io/)

# 2 Getting started 

## 2.1 Dependencies & Installation  

**(1) traj_gen**

This package is for generating smooth path after preplanning (skeleton of path).

**(2) rotors_simulator**

This package simulates MAV dynamics and controls. 
https://github.com/ethz-asl/rotors_simulator
It is required to simulate the algorithm with MAV equipped with vision sensor(mode 2, see section 3.2).  

# 3 Usage 
This package provides two modes: 1) simulation without gazebo, 2) without gazebo.   
The first mode can focus on the tracking algorithm without real MAV simulator 

## 3.1 Simulation without gazebo 

### (1) one-shot mode 

### (2) 

## 3.2 Simulation with gazebo (still developing)

# 4. ROS Node API 

## 4.1  Published and subscribed topics


## 4.2 Parameters  
