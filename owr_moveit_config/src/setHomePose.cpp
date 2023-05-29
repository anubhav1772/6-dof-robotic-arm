#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>

// Boost headers
#include <boost/shared_ptr.hpp>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
const std::string ARM_PLANNING_GROUP = "arm_manipulator";
//const std::string GRIPPER_PLANNING_GROUP = "gripper";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "setHomePose");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1); 
  spinner.start();

  MoveGroupPtr arm_move_group = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP));
  // arm_move_group->setPoseReferenceFrame(frame_id_);
  //MoveGroupPtr gripper_move_group = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(GRIPPER_PLANNING_GROUP));

  arm_move_group->setNamedTarget("HOME");
  //gripper_move_group->setNamedTarget("CLOSED");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  arm_move_group->move();

  //ros::Duration(2.0).sleep();

  //gripper_move_group->move();

  spinner.stop();

  ros::shutdown();
  return 0;
}
    
