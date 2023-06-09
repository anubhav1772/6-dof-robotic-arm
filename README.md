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

### Disable Collision in Rviz

<table>
  <tr>
    <td>Filtered PointCloud Data</td>
     <td>Generated Octomap</td>
     <td>Collision disabled for 3 objects</td>
  </tr>
  <tr>
    <td><img src="https://drive.google.com/uc?export=view&id=1ibvJi3YhnAwvcO17GX6TZhM4_OUAAdyE" width=350 height=150></td>
    <td><img src="https://drive.google.com/uc?export=view&id=1aIRiaYav0WUM5_HFjkn2oanGViiooOTX" width=350 height=150></td>
    <td><img src="https://drive.google.com/uc?export=view&id=1YU-VBAozAQOEHTdl8ZJHobHELmK8KhzD" width=350 height=150></td>
  </tr>
 </table>

### Dependencies
---

pcl-1.10.0

ROS - Noetic

Gazebo version 11.11.0
