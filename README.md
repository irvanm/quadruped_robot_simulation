# Quadruped Robot ODrive-ROS
Quadruped robot controller using ODrive motor drivers in Robot Operating System (ROS). A replication of Boston Dynamics Spot robot.

## Installation
```
# create workspace
$ mkdir quadruped_robot_ws && cd quadruped_robot_ws
$ mkdir src && cd src

# clone this repo
$ git clone https://github.com/irvanm/quadruped_robot_simulation

# build the workspace
$ cd ..
$ catkin_make
```

maxliebao is the package for simulation in gazebo. it used this repo https://github.com/F-sf/quadrupedal_gazebo_sim
hardware_pkg is the package for interfacing to Odrive using USB. It runs python code.
