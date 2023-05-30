# 6-dof-robotic-arm
6 DOF Robotic Arm (ROS + Gazebo)


### Installation
---

Clone the repository using:

    git clone https://github.com/anubhav1772/6-dof-robotic-arm.git

Run catkin_make in your ROS source directory

    $ cd ~/catkin_ws
    $ catkin_make

Start the simulation using:

    $ roslaunch owr_gazebo owr_spawn.launch

Launch moveit and rviz:

    $ roslaunch owr_moveit_config owr_simulation_execution.launch

### Dependencies
---

pcl-1.10.0

ROS - Noetic

Gazebo version 11.11.0
