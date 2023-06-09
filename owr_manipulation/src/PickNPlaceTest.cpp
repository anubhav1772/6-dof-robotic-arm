#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometric_shapes/shape_operations.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class PickNPlaceTest
{
public:
    explicit PickNPlaceTest(ros::NodeHandle nh);
    ~PickNPlaceTest();

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
    const std::string OBJECT_8_MESH_PATH = "package://owr_gazebo/models/biscuits/meshes/biscuits.dae";
    const std::string OBJECT_9_MESH_PATH = "package://owr_gazebo/models/cracker/meshes/textured.dae";

    moveit::planning_interface::MoveGroupInterface arm_move_group;
    moveit::planning_interface::MoveGroupInterface gripper_move_group;

    const robot_state::JointModelGroup *arm_joint_model_group;
    const robot_state::JointModelGroup *gripper_joint_model_group;

    // Define PlanningSceneInterface object to add and remove collision objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr;
    ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Publisher world_joint_pub;

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

    /*
        Functions for gripper actuation
        close_gripper = 0; open gripper
                      = 1; close gripper
    */
    bool OperateGripper(const bool &close_gripper, float value);

    void SetupCollisionObject(const std::string &object_id,
                              const std::string &mesh_path,
                              const geometry_msgs::Pose &object_pose,
                              moveit_msgs::CollisionObject &collision_object);

    tf2::Quaternion RPYToQuaternion(float R, float P, float Y);
};

PickNPlaceTest::PickNPlaceTest(ros::NodeHandle nh)
    : nh_(nh),
      arm_move_group(ARM_PLANNING_GROUP),
      gripper_move_group(GRIPPER_PLANNING_GROUP)
{

    // arm_move_group.setPlanningFrame("base_link");
    // Pointer to JointModelGroup for improved performance.
    arm_joint_model_group = arm_move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
    gripper_joint_model_group = gripper_move_group.getCurrentState()->getJointModelGroup(GRIPPER_PLANNING_GROUP);

    ROS_INFO_STREAM(gripper_move_group.getCurrentState());

    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("ARM MOTION", "Reference frame: %s", arm_move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("ARM MOTION", "End effector link: %s", arm_move_group.getEndEffectorLink().c_str());

    robot_model_loader = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    std::vector<geometry_msgs::Pose> mesh_pose_list;
    geometry_msgs::Pose target_mesh_pose;

    // Mesh poses
    // cracker 
    OBJECT_MESH_PATH_LIST.push_back(OBJECT_9_MESH_PATH);

    // cracker pose
    // 0.854077 -0.02371 1.234 -0.040365 -0.000621 -1.5396
    tf2::Quaternion qt = RPYToQuaternion(-0.040365, -0.000621, -1.539607);
    target_mesh_pose.position.x = 0.854077;
    target_mesh_pose.position.y = -0.02371;
    target_mesh_pose.position.z = 1.234;
    target_mesh_pose.orientation.w = qt.getW();
    target_mesh_pose.orientation.x = qt.getX();
    target_mesh_pose.orientation.y = qt.getY();
    target_mesh_pose.orientation.z = qt.getZ();
    mesh_pose_list.push_back(target_mesh_pose);

    // Add object to the scene
    std::vector<moveit_msgs::CollisionObject> target_object_list;
    for (size_t i = 0; i < 1; ++i)
    {
        moveit_msgs::CollisionObject target_collision_object;
        std::string target_id;
        target_id = "target" + std::to_string(i);
        SetupCollisionObject(target_id, OBJECT_MESH_PATH_LIST[i], mesh_pose_list[i], target_collision_object);
        target_object_list.push_back(target_collision_object);
    }

    planning_scene_interface.applyCollisionObjects(collision_objects);
    ros::Duration(0.1).sleep();

    // Allow collisions between the gripper and the objects to be able to grasp it
    planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
    collision_detection::AllowedCollisionMatrix& acm = ls->getAllowedCollisionMatrixNonConst();
    for (size_t i = 0; i < 1; ++i)
    {
        std::string target_id;
        target_id = "target" + std::to_string(i);
        
        // setEntry() allows us to specify whether collisions are allowed or not between a 
        //            pair of objects/links.
        // setEntry(link1, link2, allowed)
        // allowed=true allows collision between links link1 and link2
        acm.setEntry(target_id, "left_finger_tip", true);
        acm.setEntry(target_id, "left_inner_knuckle", true);
        acm.setEntry(target_id, "left_inner_finger", true);
        acm.setEntry(target_id, "left_outer_finger", true);
        acm.setEntry(target_id, "right_finger_tip", true);
        acm.setEntry(target_id, "right_inner_knuckle", true);
        acm.setEntry(target_id, "right_inner_finger", true);
        acm.setEntry(target_id, "right_outer_finger", true);
    }

    std::cout<<"\nAllowedCollisionMatrix:\n";
    acm.print(std::cout);

    moveit_msgs::PlanningScene diff_scene;
    ls->getPlanningSceneDiffMsg(diff_scene);

    planning_scene_interface.applyPlanningScene(diff_scene);
    ros::Duration(0.1).sleep();

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    geometry_msgs::Pose target_pose;

    target_pose.orientation.w = 0.4947743972838332;
    target_pose.orientation.x = -0.4952133362195981;
    target_pose.orientation.y = 0.5001244266339454;
    target_pose.orientation.z = -0.5097426853881694;
    target_pose.position.x = 0.61245;
    target_pose.position.y = -0.01091;
    target_pose.position.z = 1.43022;

    // set starting pose
    arm_move_group.setStartStateToCurrentState();
    // set target pose
    arm_move_group.setPoseTarget(target_pose);
    // 5 cm (0.05m)
    arm_move_group.setGoalTolerance(0.05);
    bool success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to target 1: %s", success ? "SUCCEEDED" : "FAILED");

    arm_move_group.execute(arm_plan);

    target_pose.orientation.w = 0.4947874914183782;
    target_pose.orientation.x = -0.49524148204092966;
    target_pose.orientation.y = 0.5001619108730555;
    target_pose.orientation.z = -0.5096658471132098;
    target_pose.position.x = 0.61342;
    target_pose.position.y = -0.01187;
    target_pose.position.z = 1.33021;

    arm_move_group.setStartStateToCurrentState();
    arm_move_group.setGoalTolerance(0.05);
    arm_move_group.setPoseTarget(target_pose);
    success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to target 2: %s", success ? "SUCCEEDED" : "FAILED");

    arm_move_group.execute(arm_plan);

    // close_gripper == false means open gripper action
    // value can decrease from 0.775 -> 0.04
    // OperateGripper(false, 0.4);   

    // close_gripper == true means close gripper action
    // value can decrease from 0.04 -> 0.775 
    OperateGripper(true, 0.55);

    moveit_msgs::AttachedCollisionObject aco;
    aco.object.id = collision_objects[0].id;
    aco.link_name = "left_finger_tip";
    // (touch_links) field allows us to specify a set of links that should be excluded 
    //               from collision checking, even if they come into contact with the attached object.
    aco.touch_links.push_back("left_inner_knuckle");
    //aco.touch_links.push_back("left_inner_finger");
    aco.touch_links.push_back("right_finger_tip");
    aco.touch_links.push_back("right_inner_knuckle");
    //aco.touch_links.push_back("right_inner_finger");
    
    aco.object.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface.applyAttachedCollisionObject(aco);

    target_pose.orientation.w = 0.6886415929006499;
    target_pose.orientation.x = -0.6942511017470506;
    target_pose.orientation.y = 0.14397427108592917;
    target_pose.orientation.z = -0.1518537899281326;
    target_pose.position.x = 0.23252;
    target_pose.position.y = 0.20698;
    target_pose.position.z = 1.41375;
  
    arm_move_group.setStartStateToCurrentState();
    arm_move_group.setGoalTolerance(0.05);
    arm_move_group.setPoseTarget(target_pose);
    success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to target 3 (place object): %s", success ? "SUCCEEDED" : "FAILED");
}

PickNPlaceTest::~PickNPlaceTest() {}

bool PickNPlaceTest::OperateGripper(const bool &close_gripper, float value)
{
    // RobotState contains the current position/velocity/acceleration data
    moveit::core::RobotStatePtr gripper_current_state = gripper_move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> gripper_joint_positions;
    gripper_current_state->copyJointGroupPositions(gripper_joint_model_group, gripper_joint_positions);

    //ROS_INFO("No. of  gripper joint is %ld",gripper_joint_positions.size());

    gripper_joint_positions[0] = value;
    gripper_move_group.setJointValueTarget(gripper_joint_positions);
    //ros::Duration(1.0).sleep();
    
    bool success;
    if (close_gripper)
    { 
        // close_gripper != false or close_gripper == true
        // gripper_joint_positions[0] = 0.775;
        ROS_INFO_STREAM("Initiating close gripper action...");
        success = static_cast<bool>(gripper_move_group.move());
        ROS_INFO("Close gripper action is %s",(success==true)? "success":"failure");
    }
    else
    {
        // close_gripper == false or open_gripper == true
        // gripper_joint_positions[0] = 0.04;
        ROS_INFO_STREAM("Initiating open gripper action...");
        success = static_cast<bool>(gripper_move_group.move());
        ROS_INFO("Open gripper action is %s",(success==true)? "success":"failure");
    }  
  return success;
}

void PickNPlaceTest::SetupCollisionObject(const std::string &object_id,
                                      const std::string &mesh_path,
                                      const geometry_msgs::Pose &object_pose,
                                      moveit_msgs::CollisionObject &collision_object)
{

    collision_object.header.frame_id = arm_move_group.getPlanningFrame();
    collision_object.id = object_id;

    shapes::Mesh *m = shapes::createMeshFromResource(mesh_path);

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

    collision_objects.push_back(collision_object);
}

tf2::Quaternion PickNPlaceTest::RPYToQuaternion(float R, float P, float Y)
{
    tf2::Matrix3x3 mat;
    mat.setEulerYPR(Y, P, R);

    tf2::Quaternion quat;
    mat.getRotation(quat);

    return quat;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PickNPlaceTest");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    PickNPlaceTest pickplace(nh);

    //ros::waitForShutdown();
    ros::shutdown();
    return 0;
}
