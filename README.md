# Quadruped Robot ODrive-ROS
Quadruped robot controller using ODrive motor drivers in Robot Operating System (ROS). A replication of Boston Dynamics Spot robot.

## Installation
```
# create workspace
$ mkdir quadruped_robot_ws && cd quadruped_robot_ws
$ mkdir src && cd src

# clone this repo
$ git clone https://github.com/irvanm/quadruped-robot-odrive-ros.git

# build the workspace
$ cd ..
$ catkin_make
```