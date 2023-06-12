#include <string>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometric_shapes/shape_operations.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <eigen_conversions/eigen_msg.h>

class ArmMotion{
    public:
        explicit ArmMotion(ros::NodeHandle nh);
        ~ArmMotion();
    
    private:
        ros::NodeHandle nh_;

        const std::string ARM_PLANNING_GROUP = "arm_manipulator";
        const std::string GRIPPER_PLANNING_GROUP = "gripper";

        const std::string DROPBOX_MESH_PATH = "package://owr_gazebo/models/dropbox/meshes/dropbox.dae";

        std::vector<std::string> OBJECT_MESH_PATH_LIST;
        const std::string OBJECT_1_MESH_PATH = "package://owr_gazebo/models/tomato_sauce/meshes/tomato_sauce.dae";
        const std::string OBJECT_2_MESH_PATH = "package://owr_gazebo/models/soap/meshes/soap.dae";
        const std::string OBJECT_3_MESH_PATH = "package://owr_gazebo/models/eraser/meshes/eraser.dae";
        const std::string OBJECT_4_MESH_PATH = "package://owr_gazebo/models/soap2/meshes/soap2.dae";
        const std::string OBJECT_5_MESH_PATH = "package://owr_gazebo/models/plastic_cup/meshes/plastic_cup.dae";
        const std::string OBJECT_6_MESH_PATH = "package://owr_gazebo/models/soda_can/meshes/soda_can.dae";
        const std::string OBJECT_7_MESH_PATH = "package://owr_gazebo/models/mustard/meshes/mustard.dae";
        const std::string OBJECT_8_MESH_PATH = "package://owr_gazebo/models/bowl/meshes/bowl.dae";
        const std::string OBJECT_9_MESH_PATH = "package://owr_gazebo/models/cracker/meshes/textured.dae";

        moveit::planning_interface::MoveGroupInterface arm_move_group;
        moveit::planning_interface::MoveGroupInterface gripper_move_group;

        const robot_state::JointModelGroup *arm_joint_model_group;
        const robot_state::JointModelGroup *gripper_joint_model_group;

        // moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr;
        ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

        // Define PlanningSceneInterface object to add and remove collision objects
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        ros::Publisher world_joint_pub;

        /*
            Functions for gripper actuation
            close_gripper = 0; open gripper
                          = 1; close gripper
        */
        bool OperateGripper(const bool &close_gripper);

        void SetupCollisionObject(const std::string &object_id,
                                  const std::string &mesh_path,
                                  const geometry_msgs::Pose &object_pose,
                                  moveit_msgs::CollisionObject &collision_object);

        tf2::Quaternion RPYToQuaternion(float R, float P, float Y);
}; 

ArmMotion::ArmMotion(ros::NodeHandle nh)
  : nh_(nh),
    arm_move_group(ARM_PLANNING_GROUP),
    gripper_move_group(GRIPPER_PLANNING_GROUP){
      
    // arm_move_group.setPlanningFrame("base_link");
    // Pointer to JointModelGroup for improved performance.
    arm_joint_model_group = arm_move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
    gripper_joint_model_group = gripper_move_group.getCurrentState()->getJointModelGroup(GRIPPER_PLANNING_GROUP);
      
    ROS_INFO_STREAM(gripper_move_group.getCurrentState());

    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("ARM MOTION", "Reference frame: %s", arm_move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("ARM MOTION", "End effector link: %s", arm_move_group.getEndEffectorLink().c_str());

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    geometry_msgs::Pose target_pose;

    target_pose.orientation.w = 0.37059517429997146;
    target_pose.orientation.x = -0.6021321052069217;
    target_pose.orientation.y = 0.6022171464773562;
    target_pose.orientation.z = -0.3707164052929471;
    target_pose.position.x = 0.491;
    target_pose.position.y = -0.047;
    target_pose.position.z = 1.581;

    // set starting pose
    arm_move_group.setStartStateToCurrentState();
    // set target pose
    arm_move_group.setPoseTarget(target_pose);
    arm_move_group.setGoalTolerance(0.01);
    bool success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to target 1: %s", success ? "SUCCEEDED" : "FAILED");

    arm_move_group.execute(arm_plan);

    target_pose.orientation.w = 0.37173772501897173;
    target_pose.orientation.x = -0.6023516784804275;
    target_pose.orientation.y = 0.5983131228259329;
    target_pose.orientation.z = -0.3755062266911314;
    target_pose.position.x = 0.500886;
    target_pose.position.y = 0.053937;
    target_pose.position.z = 1.5773409;

    arm_move_group.setPoseTarget(target_pose);
    success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to target 2: %s", success ? "SUCCEEDED" : "FAILED");

    arm_move_group.execute(arm_plan);

    target_pose.orientation.w = 0.3690263551567426;
    target_pose.orientation.x = -0.601687141809745;
    target_pose.orientation.y = 0.601910625422726;
    target_pose.orientation.z = -0.373491273771921;
    target_pose.position.x = 0.57585;
    target_pose.position.y = 0.04420;
    target_pose.position.z = 1.54783;

    arm_move_group.setPoseTarget(target_pose);
    success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to target 3: %s", success ? "SUCCEEDED" : "FAILED");

    arm_move_group.execute(arm_plan);
}

ArmMotion::~ArmMotion(){}

bool ArmMotion::OperateGripper(const bool &close_gripper){

  ROS_INFO_STREAM("Initiating close gripper action...");

  // RobotState contains the current position/velocity/acceleration data
  moveit::core::RobotStatePtr gripper_current_state = gripper_move_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group, gripper_joint_positions);

  ROS_INFO("No. of  gripper joint is %ld",gripper_joint_positions.size());

  // Set finger joint values
  if (close_gripper)
  { // close_gripper != false or close_gripper == true
    gripper_joint_positions[0] = 0.775;
  }else{
    // close_gripper == false or open_gripper == true
    gripper_joint_positions[0] = 0.0;
  }

  gripper_move_group.setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.5).sleep();

  bool success = static_cast<bool>(gripper_move_group.move());
  ROS_INFO("Close gripper action is %s",(success==true)? "success":"failure");
  return success;
}

void ArmMotion::SetupCollisionObject(const std::string &object_id,
                                    const std::string &mesh_path,
                                    const geometry_msgs::Pose &object_pose,
                                    moveit_msgs::CollisionObject &collision_object){

  collision_object.header.frame_id = arm_move_group.getPlanningFrame();
  collision_object.id = object_id;

  shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);

  ROS_DEBUG_STREAM(object_id << " mesh loaded");

  shape_msgs::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m, object_mesh_msg);
  object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);
  collision_object.meshes.resize(1);
  collision_object.mesh_poses.resize(1);
  collision_object.meshes[0] = object_mesh;

  collision_object.mesh_poses[0].position = object_pose.position;
  collision_object.mesh_poses[0].orientation = object_pose.orientation;

  collision_object.meshes.push_back(object_mesh);
  collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
  collision_object.operation = collision_object.ADD;
}

tf2::Quaternion ArmMotion::RPYToQuaternion(float R, float P, float Y){
  tf2::Matrix3x3 mat;
  mat.setEulerYPR(Y,P,R);

  tf2::Quaternion quat;
  mat.getRotation(quat);

  return quat;
}


int main(int argc, char **argv){

  ros::init(argc, argv, "ArmMotion");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ArmMotion arm_motion(nh);
  return 0;
}
