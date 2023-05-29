#include <exception>
#include <string>
#include <math.h>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>


// action interface type
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;


// Create a ROS action client to move OW arm
void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

  actionClient.reset( new arm_control_client("/arm_manipulator_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_manipulator_controller action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm manipulator controller action server not available");
}


// Generates a simple trajectory with two waypoints to move OW arm 
void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("BJ");
    goal.trajectory.joint_names.push_back("SJ");
    goal.trajectory.joint_names.push_back("EJ");
    goal.trajectory.joint_names.push_back("W1J");
    goal.trajectory.joint_names.push_back("W2J");
    goal.trajectory.joint_names.push_back("W3J");
    
    // 2 waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int index = 0;
    goal.trajectory.points[index].positions.resize(6);
    goal.trajectory.points[index].positions[0] = M_PI/2;
    goal.trajectory.points[index].positions[1] = 0.0;
    goal.trajectory.points[index].positions[2] = 0.0;
    goal.trajectory.points[index].positions[3] = 0.0;
    goal.trajectory.points[index].positions[4] = 0.0;
    goal.trajectory.points[index].positions[5] = 0.0;

    // Velocities
    goal.trajectory.points[index].velocities.resize(6);
    for (int j = 0; j < 6; ++j)
    {
        goal.trajectory.points[index].velocities[j] = 0.0;
    }
    // To be reached 5 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(5.0);

    // Second trajectory point
    index += 1;
    goal.trajectory.points[index].positions.resize(6);
    goal.trajectory.points[index].positions[0] = 0.0;
    goal.trajectory.points[index].positions[1] = 0.0;
    goal.trajectory.points[index].positions[2] = M_PI/2;
    goal.trajectory.points[index].positions[3] = 0.0;
    goal.trajectory.points[index].positions[4] = 0.0;
    goal.trajectory.points[index].positions[5] = 0.0;
    // Velocities
    goal.trajectory.points[index].velocities.resize(6);
    for (int j = 0; j < 6; ++j)
    {
        goal.trajectory.points[index].velocities[j] = 0.0;
    }
    // To be reached 10 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(10.0);
    
    /**
    // Third trajectory point
    // Positions
    index += 1;
    goal.trajectory.points[index].positions.resize(6);
    goal.trajectory.points[index].positions[0] = 0.0;
    goal.trajectory.points[index].positions[1] = M_PI/2;
    goal.trajectory.points[index].positions[2] = 0.0;
    goal.trajectory.points[index].positions[3] = 0.0;
    goal.trajectory.points[index].positions[4] = M_PI/2;
    goal.trajectory.points[index].positions[5] = 0.0;
    // Velocities
    goal.trajectory.points[index].velocities.resize(6);
    for (int j = 0; j < 6; ++j)
    {
        goal.trajectory.points[index].velocities[j] = 0.5;
    }
    // To be reached 4 seconds after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(15.0);

    // Fourth trajectory point
    // Positions
    index += 1;
    goal.trajectory.points[index].positions.resize(6);
    goal.trajectory.points[index].positions[0] = 0.0;
    goal.trajectory.points[index].positions[1] = 0.0;
    goal.trajectory.points[index].positions[2] = 0.0;
    goal.trajectory.points[index].positions[3] = 0.0;
    goal.trajectory.points[index].positions[4] = 0.0;
    goal.trajectory.points[index].positions[5] = 0.0;
    // Velocities
    goal.trajectory.points[index].velocities.resize(6);
    for (int j = 0; j < 6; ++j)
    {
        goal.trajectory.points[index].velocities[j] = 0.5;
    }
    // To be reached 6 seconds after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(20.0);
    **/
}


// Entry point
int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "joint_trajectory_control");

    ROS_INFO("Starting joint_trajectory_control application ...");
    
    // Precondition: Valid clock
    ros::NodeHandle nh;
    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
    {
        ROS_FATAL("Timed-out waiting for valid time.");
        return EXIT_FAILURE;
    }

    // Create an arm controller action client to move the OW arm
    arm_control_client_Ptr ArmClient;
    createArmClient(ArmClient);

    // Generates the goal for the OW arm
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    waypoints_arm_goal(arm_goal);

    // Sends the command to start the given trajectory 1s from now
    arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    ArmClient->sendGoal(arm_goal);

    // Wait for trajectory execution
    while(!(ArmClient->getState().isDone()) && ros::ok())
    {
        ros::Duration(12.0).sleep(); // sleep for four seconds
    }

    return EXIT_SUCCESS;
}