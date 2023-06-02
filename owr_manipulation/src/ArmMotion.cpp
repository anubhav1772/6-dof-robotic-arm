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

      /*
      * rviz visualization:
      * Setup MoveItVisualTools for visualizing collision objects, robot,
      * and trajectories in Rviz
      */
      /*
      moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr;
      visual_tools_ptr.reset(new moveit_visual_tools::MoveItVisualTools("world"));
      visual_tools_ptr->deleteAllMarkers();

      // Load RemoteControl for step-by-step progression
      visual_tools_ptr->loadRemoteControl();
      */

      // Create text marker for displaying current state
      // Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
      // text_pose.translation().z() = 1.5;
      // visual_tools_ptr->publishText(text_pose, "Welcome to Advance Pick and Place project",
      //                               rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
      // Publish messages to rviz
      // visual_tools_ptr->trigger();
      // visual_tools_ptr->prompt("next step");

      /*
      * Collision Objects:
      * Create an object list and populate it with shelf and bin objects
      * Then insert objects in scene for collision avoidance and interaction
      */
      /*
      std::vector<moveit_msgs::CollisionObject> collision_object_list;
      std::vector<std::string> object_ids;
      moveit_msgs::CollisionObject dropbox_collision_object;

      // Define pose for the objects (specified relative to /base_link)
      geometry_msgs::Pose dropbox_mesh_pose;  

      dropbox_mesh_pose.position.x = -0.745301;
      dropbox_mesh_pose.position.y = -0.03408;
      dropbox_mesh_pose.position.z = 0.612293;

      tf2::Quaternion qt = RPYToQuaternion(0, 0, 0);
      dropbox_mesh_pose.orientation.w = qt.getW();
      dropbox_mesh_pose.orientation.x = qt.getX();
      dropbox_mesh_pose.orientation.y = qt.getY();
      dropbox_mesh_pose.orientation.z = qt.getZ();

      ROS_INFO_STREAM(DROPBOX_MESH_PATH);
      SetupCollisionObject("dropbox", DROPBOX_MESH_PATH, dropbox_mesh_pose,
                            dropbox_collision_object);

      collision_object_list.push_back(dropbox_collision_object); 

      // Add the object list to the world scene
      planning_scene_interface.addCollisionObjects(collision_object_list);

      ros::Duration(3.0).sleep();
      */
      //Rotate in place to capture collision map from the sides
      // std_msgs::Float64 world_joint_value;

      // world_joint_value.data = -1.57;
      // world_joint_pub.publish(world_joint_value);
      // ros::Duration(1.0).sleep();
      // visual_tools_ptr->prompt("next step");

      // world_joint_value.data = 1.57;
      // world_joint_pub.publish(world_joint_value);
      // ros::Duration(1.0).sleep();
      // visual_tools_ptr->prompt("next step");

      // world_joint_value.data = 0;
      // world_joint_pub.publish(world_joint_value);
      // ros::Duration(1.0).sleep();
      // visual_tools_ptr->prompt("next step");

      /*
      //Target object pick pose
      std::vector<geometry_msgs::Pose> pose_list, drop_list, mesh_pose_list;
      geometry_msgs::Pose target_pose, drop_pose, target_mesh_pose;

      // a given drop pose 
      drop_pose.orientation.w = 0.494481597524;
      drop_pose.orientation.x = 0.59360155453;
      drop_pose.orientation.y = -0.486540200164;
      drop_pose.orientation.z = -0.407926191602;
      drop_pose.position.x = -0.572936675058;
      drop_pose.position.y = 0.0893864826716;
      drop_pose.position.z = 0.92520259471;
      drop_list.push_back(drop_pose);

      // drop pose of biscuit
      // drop_pose.orientation.w = 1.0;
      // drop_pose.position.x = -0.592;
      // drop_pose.position.y = -0.205;
      // drop_pose.position.z = 0.984;
      // drop_list.push_back(drop_pose);

      // drop_pose.orientation.w = 1.0;
      // drop_pose.position.x = -0.03;
      // drop_pose.position.y = -0.65;
      // drop_pose.position.z = 1.0;
      // drop_list.push_back(drop_pose);

      // drop_pose.orientation.w = 1.0;
      // drop_pose.position.x = -0.1;
      // drop_pose.position.y = -0.8;
      // drop_pose.position.z = 1.0;
      // drop_list.push_back(drop_pose);

      // biscuit target pose
   
      // target_pose.orientation.w = 0.00513957824022;
      // target_pose.orientation.x = -0.00402826016725;
      // target_pose.orientation.y = -0.707066080128;
      // target_pose.orientation.z = -0.707117328446;
      // target_pose.position.x = 0.182839955222;
      // target_pose.position.y = 0.444278893027;
      // target_pose.position.z = 0.791364654825;
      // pose_list.push_back(target_pose);

      target_pose.orientation.w = 0.00602595854721;
      target_pose.orientation.x = 0.000293589357155;
      target_pose.orientation.y = -0.996191237832;
      target_pose.orientation.z = -0.08698631671;
      target_pose.position.x = 0.188795170567;
      target_pose.position.y = 0.550522295237;
      target_pose.position.z = 1.21502249626;
      pose_list.push_back(target_pose);

      // Mesh poses
      // cracker 1
      OBJECT_MESH_PATH_LIST.push_back(OBJECT_9_MESH_PATH);
      // cracker 2
      OBJECT_MESH_PATH_LIST.push_back(OBJECT_9_MESH_PATH);
      // cracker 3
      OBJECT_MESH_PATH_LIST.push_back(OBJECT_9_MESH_PATH);
      // tomato_sauce
      OBJECT_MESH_PATH_LIST.push_back(OBJECT_1_MESH_PATH);
      // mustard
      OBJECT_MESH_PATH_LIST.push_back(OBJECT_7_MESH_PATH);
     
      // cracker 1 pose
      qt = RPYToQuaternion(0.0, -0.056478, 0.0);
      target_mesh_pose.position.x = 0.836851;
      target_mesh_pose.position.y = -0.122657;
      target_mesh_pose.position.z = 1.244513;
      target_mesh_pose.orientation.w = qt.getW();
      target_mesh_pose.orientation.x = qt.getX();
      target_mesh_pose.orientation.y = qt.getY();
      target_mesh_pose.orientation.z = qt.getZ();
      mesh_pose_list.push_back(target_mesh_pose);
      
      // cracker 2 pose
      qt = RPYToQuaternion(-0.007372, 0.039691, -0.137111);
      target_mesh_pose.position.x = 0.899215;
      target_mesh_pose.position.y = 0.052025;
      target_mesh_pose.position.z = 1.232038;
      target_mesh_pose.orientation.w = qt.getW();
      target_mesh_pose.orientation.x = qt.getX();
      target_mesh_pose.orientation.y = qt.getY();
      target_mesh_pose.orientation.z = qt.getZ();
      mesh_pose_list.push_back(target_mesh_pose);
      
      // cracker 3 pose
      qt = RPYToQuaternion(-0.007372, 0.039691, -0.137111);
      target_mesh_pose.position.x = 0.804891;
      target_mesh_pose.position.y = 0.312086;
      target_mesh_pose.position.z = 1.232038;
      target_mesh_pose.orientation.w = qt.getW();
      target_mesh_pose.orientation.x = qt.getX();
      target_mesh_pose.orientation.y = qt.getY();
      target_mesh_pose.orientation.z = qt.getZ();
      mesh_pose_list.push_back(target_mesh_pose);
     
      // Tomato Sauce Can Pose
      qt = RPYToQuaternion(0, 0, 0.246588);
      target_mesh_pose.position.x = 0.919908;
      target_mesh_pose.position.y = -0.291053;
      target_mesh_pose.position.z = 1.312097;
      target_mesh_pose.orientation.w = qt.getW();
      target_mesh_pose.orientation.x = qt.getX();
      target_mesh_pose.orientation.y = qt.getY();
      target_mesh_pose.orientation.z = qt.getZ();
      mesh_pose_list.push_back(target_mesh_pose); 

      // Mustard Can Pose
      qt = RPYToQuaternion(0.064318, 0.014760, 1.518023);
      target_mesh_pose.position.x = 0.664781;
      target_mesh_pose.position.y = 0.340374;
      target_mesh_pose.position.z = 1.39052;
      target_mesh_pose.orientation.w = qt.getW();
      target_mesh_pose.orientation.x = qt.getX();
      target_mesh_pose.orientation.y = qt.getY();
      target_mesh_pose.orientation.z = qt.getZ();
      mesh_pose_list.push_back(target_mesh_pose);

      //Add object to the scene
      std::vector<moveit_msgs::CollisionObject> target_object_list;
      for(size_t i=0; i<3;++i)
      {
        moveit_msgs::CollisionObject target_collision_object;
        std::string target_id;
        target_id = "target"+ std::to_string (i);

        SetupCollisionObject(target_id, OBJECT_MESH_PATH_LIST[i], mesh_pose_list[i], target_collision_object);

        target_object_list.push_back(target_collision_object);
      }

      // Add the object list to the world scene
      planning_scene_interface.addCollisionObjects(target_object_list);
      ROS_INFO("Added Target to the world");

      // Allow MoveGroup to add the collision objects in the world
      ros::Duration(1.0).sleep();

      // Move to ZERO position
      arm_move_group.setJointValueTarget(arm_move_group.getNamedTargetValues("ZERO"));

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (arm_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success == false)
        return;
      
      */

      //arm_move_group.move();
      
      /**
      // set starting pose
      arm_move_group.setStartStateToCurrentState();
      // set target pose
      arm_move_group.setPoseTarget(pose_list[0]);
      success = static_cast<bool>(arm_move_group.plan(arm_plan));
      ROS_INFO("Visualizing plan to target: %s", success ? "SUCCEEDED" : "FAILED");

      arm_move_group.execute(arm_plan);

      
      // Reach movement
      arm_move_group.setStartStateToCurrentState();
      pose_list[0].position.y = pose_list[0].position.y+0.06632;
      arm_move_group.setPoseTarget(pose_list[0]);
      success = static_cast<bool>(arm_move_group.plan(arm_plan));
      ROS_INFO("Visualizing plan to target: %s", success ? "SUCCEEDED" : "FAILED");

      arm_move_group.execute(arm_plan);

      arm_move_group.setStartStateToCurrentState();
      pose_list[0].position.z = pose_list[0].position.z-0.338;
      arm_move_group.setPoseTarget(pose_list[0]);
      success = static_cast<bool>(arm_move_group.plan(arm_plan));
      ROS_INFO("Visualizing plan to target: %s", success ? "SUCCEEDED" : "FAILED");

      arm_move_group.execute(arm_plan);

      // Remove object from the scene
      object_ids.push_back(target_object_list[0].id);
      planning_scene_interface.removeCollisionObjects(object_ids);

      // Close Gripper
      OperateGripper(true);
      ros::Duration(3.0).sleep();

      // drop the biscuit
      arm_move_group.setStartStateToCurrentState();
      arm_move_group.setPoseTarget(drop_list[0]);
      success = static_cast<bool>(arm_move_group.plan(arm_plan));
      ROS_INFO("Visualizing plan to target: %s", success ? "SUCCEEDED" : "FAILED");

      arm_move_group.execute(arm_plan);
      // Open Gripper
      OperateGripper(false);
      **/

      // move to a valid random pose
      // ERROR - MoveIt! unable to sample any valid states for goal tree
      // FIX - https://groups.google.com/g/moveit-users/c/yRthi64affg?pli=1
      moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
      geometry_msgs::Pose target_pose;
   
      target_pose.orientation.w = 0.5233436774;
      target_pose.orientation.x = 0.7860557390;
      target_pose.orientation.y = -0.3270965294;
      target_pose.orientation.z = -0.0351515399;
      target_pose.position.x = 0.666;
      target_pose.position.y = -0.158;
      target_pose.position.z = 1.124;

      // set starting pose
      arm_move_group.setStartStateToCurrentState();
      // set target pose
      arm_move_group.setPoseTarget(target_pose);
      arm_move_group.setGoalTolerance(0.01);
      bool success = static_cast<bool>(arm_move_group.plan(arm_plan));
      ROS_INFO("Plan to target: %s", success ? "SUCCEEDED" : "FAILED");

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
    