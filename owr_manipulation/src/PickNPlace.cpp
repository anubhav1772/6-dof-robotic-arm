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

class PickNPlace
{
public:
    explicit PickNPlace(ros::NodeHandle nh);
    ~PickNPlace();

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
    bool OperateGripper(const bool &close_gripper);

    void SetupCollisionObject(const std::string &object_id,
                              const std::string &mesh_path,
                              const geometry_msgs::Pose &object_pose,
                              moveit_msgs::CollisionObject &collision_object);

    tf2::Quaternion RPYToQuaternion(float R, float P, float Y);
};

PickNPlace::PickNPlace(ros::NodeHandle nh)
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
    // cracker 1
    OBJECT_MESH_PATH_LIST.push_back(OBJECT_9_MESH_PATH);
    // cracker 2
    OBJECT_MESH_PATH_LIST.push_back(OBJECT_9_MESH_PATH);
    // cracker 3
    OBJECT_MESH_PATH_LIST.push_back(OBJECT_9_MESH_PATH);
    // Biscuit
    OBJECT_MESH_PATH_LIST.push_back(OBJECT_8_MESH_PATH);
    // Soap
    OBJECT_MESH_PATH_LIST.push_back(OBJECT_4_MESH_PATH);

    // cracker 1 pose
    // [0.794093 -0.11269 1.242096 -0.055011 -0.012796 1.342494]
    tf2::Quaternion qt = RPYToQuaternion(-0.055011, -0.012796, 1.342494);
    target_mesh_pose.position.x = 0.794093;
    target_mesh_pose.position.y = -0.11269;
    target_mesh_pose.position.z = 1.242096;
    target_mesh_pose.orientation.w = qt.getW();
    target_mesh_pose.orientation.x = qt.getX();
    target_mesh_pose.orientation.y = qt.getY();
    target_mesh_pose.orientation.z = qt.getZ();
    mesh_pose_list.push_back(target_mesh_pose);

    // cracker 2 pose
    // [0.851959 0.04419 1.233959 -0.040091 0.004737 -1.406676]
    qt = RPYToQuaternion(-0.040091, 0.004737, -1.406676);
    target_mesh_pose.position.x = 0.851959;
    target_mesh_pose.position.y = 0.04419;
    target_mesh_pose.position.z = 1.233959;
    target_mesh_pose.orientation.w = qt.getW();
    target_mesh_pose.orientation.x = qt.getX();
    target_mesh_pose.orientation.y = qt.getY();
    target_mesh_pose.orientation.z = qt.getZ();
    mesh_pose_list.push_back(target_mesh_pose);

    // cracker 3 pose
    // [0.812623 0.251395 1.23184 -0.028299 0.028793 -0.730383]
    qt = RPYToQuaternion(-0.028299, 0.028793, -0.730383);
    target_mesh_pose.position.x = 0.812623;
    target_mesh_pose.position.y = 0.251395;
    target_mesh_pose.position.z = 1.23184;
    target_mesh_pose.orientation.w = qt.getW();
    target_mesh_pose.orientation.x = qt.getX();
    target_mesh_pose.orientation.y = qt.getY();
    target_mesh_pose.orientation.z = qt.getZ();
    mesh_pose_list.push_back(target_mesh_pose);

    // Biscuit Pose
    // [0.659679 0.34268 1.220194 0.000001 -0.000004 -1.560427]
    qt = RPYToQuaternion(0.000001, -0.000004, -1.560427);
    target_mesh_pose.position.x = 0.659679;
    target_mesh_pose.position.y = 0.34268;
    target_mesh_pose.position.z = 1.220194;
    target_mesh_pose.orientation.w = qt.getW();
    target_mesh_pose.orientation.x = qt.getX();
    target_mesh_pose.orientation.y = qt.getY();
    target_mesh_pose.orientation.z = qt.getZ();
    mesh_pose_list.push_back(target_mesh_pose);

    // Soap Pose
    // [0.64636 -0.257029 1.222691 0.000026 0.000019 -2.035517]
    qt = RPYToQuaternion(0.000026, 0.000019, -2.035517);
    target_mesh_pose.position.x = 0.64636;
    target_mesh_pose.position.y = -0.257029;
    target_mesh_pose.position.z = 1.222691;
    target_mesh_pose.orientation.w = qt.getW();
    target_mesh_pose.orientation.x = qt.getX();
    target_mesh_pose.orientation.y = qt.getY();
    target_mesh_pose.orientation.z = qt.getZ();
    mesh_pose_list.push_back(target_mesh_pose);

    // Add object to the scene
    std::vector<moveit_msgs::CollisionObject> target_object_list;
    for (size_t i = 0; i < 5; ++i)
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
    for (size_t i = 0; i < 5; ++i)
    {
        std::string target_id;
        target_id = "target" + std::to_string(i);
        acm.setEntry(target_id, "left_finger_tip", true);
        acm.setEntry(target_id, "left_inner_knucle_joint", true);
        acm.setEntry(target_id, "right_finger_tip", true);
        acm.setEntry(target_id, "right_inner_knucle_joint", true);
    }

    std::cout<<"\nAllowedCollisionMatrix:\n";
    acm.print(std::cout);

    moveit_msgs::PlanningScene diff_scene;
    ls->getPlanningSceneDiffMsg(diff_scene);

    planning_scene_interface.applyPlanningScene(diff_scene);
    ros::Duration(0.1).sleep();
    
    // Go to ZERO pose
    // set starting pose
    /*
    arm_move_group.setStartStateToCurrentState();
    arm_move_group.setNamedTarget("ZERO");
    bool success = (arm_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("STATUS", "Visualizing plan 1 (pose goal) %s", success? "":"FAILED");
    arm_move_group.execute(my_plan);
    */


    // Place the TCP (Tool Center Point, tip of the robot, End effector link -> fts_toolside) above the table
    //geometry_msgs::PoseStamped current_pose;
    //current_pose = arm_move_group.getCurrentPose("fts_toolside");

    geometry_msgs::Pose target_pose;
   
    target_pose.orientation.w = 0.37059517429997146;
    target_pose.orientation.x = -0.6021321052069217;
    target_pose.orientation.y = 0.6022171464773562;
    target_pose.orientation.z = -0.3707164052929471;
    target_pose.position.x = 0.4918;
    target_pose.position.y = -0.0477;
    target_pose.position.z = 1.5811;

    // set starting pose
    arm_move_group.setStartStateToCurrentState();
    // set target pose
    arm_move_group.setPoseTarget(target_pose);
    arm_move_group.setGoalTolerance(0.01);
    bool success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to Pose 1 %s", success ? "SUCCEEDED" : "FAILED");
    arm_move_group.execute(arm_plan);

    ros::Duration(0.2).sleep();

    target_pose.orientation.w = 0.37173772501897173;
    target_pose.orientation.x = -0.6023516784804275;
    target_pose.orientation.y = 0.5983131228259329;
    target_pose.orientation.z = -0.3755062266911314;
    target_pose.position.x = 0.500886;
    target_pose.position.y = 0.053937;
    target_pose.position.z = 1.5773409;

    arm_move_group.setStartStateToCurrentState();

    arm_move_group.setPoseTarget(target_pose);
    arm_move_group.setGoalTolerance(0.01);
    success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to Pose 2: %s", success ? "SUCCEEDED" : "FAILED");
    arm_move_group.execute(arm_plan);
}

PickNPlace::~PickNPlace() {}

bool PickNPlace::OperateGripper(const bool &close_gripper)
{

    ROS_INFO_STREAM("Initiating close gripper action...");

    // RobotState contains the current position/velocity/acceleration data
    moveit::core::RobotStatePtr gripper_current_state = gripper_move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> gripper_joint_positions;
    gripper_current_state->copyJointGroupPositions(gripper_joint_model_group, gripper_joint_positions);

    ROS_INFO("No. of  gripper joint is %ld", gripper_joint_positions.size());

    // Set finger joint values
    if (close_gripper)
    { // close_gripper != false or close_gripper == true
        gripper_joint_positions[0] = 0.775;
    }
    else
    {
        // close_gripper == false or open_gripper == true
        gripper_joint_positions[0] = 0.0;
    }

    gripper_move_group.setJointValueTarget(gripper_joint_positions);
    ros::Duration(0.1).sleep();

    bool success = static_cast<bool>(gripper_move_group.move());
    ROS_INFO("Close gripper action is %s", (success == true) ? "success" : "failure");
    return success;
}

void PickNPlace::SetupCollisionObject(const std::string &object_id,
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
    ros::Duration(0.1).sleep();
}

tf2::Quaternion PickNPlace::RPYToQuaternion(float R, float P, float Y)
{
    tf2::Matrix3x3 mat;
    mat.setEulerYPR(Y, P, R);

    tf2::Quaternion quat;
    mat.getRotation(quat);

    return quat;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PickNPlace");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    PickNPlace pickplace(nh);
    return 0;
}
