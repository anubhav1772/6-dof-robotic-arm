/**
 * 
 * Author: Anubhav Singh
 * 
 * |===============================================|
 * | rosmsg info trajectory_msgs/JointTrajectory   |
 * |===============================================|
 * | std_msgs/Header header                        |
 * |    uint32 seq                                 |
 * |    time stamp                                 |
 * |    string frame_id                            |
 * | string[] joint_names                          |
 * | trajectory_msgs/JointTrajectoryPoint[] points |
 * |    float64[] positions                        |
 * |    float64[] velocities                       |
 * |    float64[] accelerations                    |
 * |    float64[] effort                           |
 * |    duration time_from_start                   |
 * |===============================================|
 * 
 * command to publish on "/arm_manipulator_controller/command" topic 
 * and move both models (on rivz and gazebo) simultaneously.
 * rostopic pub /arm_manipulator_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["BJ", "SJ", "EJ", "W1J", "W2J", "W3J"], points: [{positions:[1.9511805914578915, 1.1164230740175043, -1.4158402476438106, -0.21242864649290638, 3.139712841112768], time_from_start: [1.0,0.0]}]}' -1
 * 
 * command to publish on "/gripper_manipulator_controller/command" topic 
 * rostopic pub /gripper_manipulator_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["gr_l", "gr_r"], points: [{positions:[1.4058, 1.4058], time_from_start: [1.0,0.0]}]}' -1
 * 
 * rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{header: {stamp: {secs: 0,nsecs: 0}}, joint_names: ["BJ", "SJ", "EJ", "W1J", "W2J", "W3J"], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {secs: 0.0, nsecs: 0.0}},{positions: [1.57, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start ...
 *
 **/


#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/time.h"
#include <iostream>
#include <math.h>

ros::Publisher arm_pub;
std::string joint_names[6] = { "BJ", "SJ", "EJ", "W1J", "W2J", "W3J" };

trajectory_msgs::JointTrajectoryPoint jointTrajectory_point(double time, int num_joints, double *joint_values){

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(num_joints);
    for(int i=0; i<num_joints; i++) {
        point.positions[i] = joint_values[i];
    }

    point.time_from_start = ros::Duration(time);
    return point;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher_node");
    ros::NodeHandle n;
    arm_pub = n.advertise<trajectory_msgs::JointTrajectory>("/arm_manipulator_controller/command",1);
    ros::Rate loop_rate(100);

    int num_joints = 6;

    trajectory_msgs::JointTrajectory traj;
    traj.header.frame_id = "base_link";
    traj.header.stamp = ros::Time::now();

    traj.joint_names.resize(num_joints);
    for(int i=0;i<num_joints;i++){
        traj.joint_names[i] = joint_names[i];
    }

    traj.points.resize(3);

    double joint_values_1[6] = {M_PI/2, M_PI/3, M_PI/2, M_PI/3, -M_PI/3, M_PI/2}; 
    traj.points[0] = jointTrajectory_point(1.0, 6, joint_values_1);

    double joint_values_2[6] = {-M_PI/2, -M_PI/3, -M_PI/2, -M_PI/2, M_PI/3, -M_PI/2}; 
    traj.points[1] = jointTrajectory_point(4.0, 6, joint_values_2);

    double joint_values_3[6] = {M_PI/3, 0.0, 0.0, 0.0, M_PI/3, 0.0}; 
    traj.points[2] = jointTrajectory_point(8.0, 6, joint_values_3);

    while(ros::ok()) {

        arm_pub.publish(traj);

        ros::spinOnce();
        loop_rate.sleep();
        
    }

    return 0;
}
