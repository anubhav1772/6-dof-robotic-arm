// MoveIt
#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

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

class PlanningScene{
    public:
        explicit PlanningScene(ros::NodeHandle nh);
        ~PlanningScene();
    
    private:
        ros::NodeHandle nh_;

        const std::string ARM_PLANNING_GROUP = "arm_manipulator";

        moveit::planning_interface::MoveGroupInterface arm_move_group;

        const robot_state::JointModelGroup *arm_joint_model_group;

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

        // moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr;
        ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

        // Define PlanningSceneInterface object to add and remove collision objects
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        ros::Publisher pub_co;
        // ros::Publisher pub_aco;

        moveit_msgs::CollisionObject co;
        moveit_msgs::AttachedCollisionObject aco;

        bool SetupCollisionObject(const std::string &object_id,
                                  const std::string &mesh_path,
                                  const geometry_msgs::Pose &object_pose,
                                  moveit_msgs::CollisionObject &collision_object);

        tf2::Quaternion RPYToQuaternion(float R, float P, float Y);

        moveit_msgs::PlanningScene planning_scene_add;
        ros::Publisher planning_scene_diff_publisher_;
        ros::ServiceClient client_get_scene_;

        moveit_msgs::GetPlanningScene srv;

        moveit_msgs::PlanningScene currentScene;
        moveit_msgs::PlanningScene newSceneDiff;

        moveit_msgs::GetPlanningScene scene_srv;
};

PlanningScene::PlanningScene(ros::NodeHandle nh)
  : nh_(nh),
    arm_move_group(ARM_PLANNING_GROUP){
      
      // arm_move_group.setPlanningFrame("base_link");
      // Pointer to JointModelGroup for improved performance.
      arm_joint_model_group = arm_move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
      
      ROS_INFO_STREAM(arm_move_group.getCurrentState());

      // visual_tools_ptr.reset(new moveit_visual_tools::MoveItVisualTools("world"));

      // We can print the name of the reference frame for this robot.
      ROS_INFO_NAMED("OWR", "Reference frame: %s", arm_move_group.getPlanningFrame().c_str());

      // We can also print the name of the end-effector link for this group.
      ROS_INFO_NAMED("OWR", "End effector link: %s", arm_move_group.getEndEffectorLink().c_str());
      
      pub_co = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
      // pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

      planning_scene_diff_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
      client_get_scene_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

      srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

      //Target object pick pose
      std::vector<geometry_msgs::Pose> mesh_pose_list;
      geometry_msgs::Pose target_mesh_pose;

      // cracker 1 pose
      tf2::Quaternion qt = RPYToQuaternion(0.0, -0.056478, 0.0);
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

      //Mesh poses
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
      ros::Duration(5.0).sleep();

      //wait until box is published (I don't think that this is really needed)
      // bool object_in_world = false;
      // while(!object_in_world)
      // {
      //     ROS_ERROR("waiting for box to appear");
      //     if (client_get_scene_.call(srv))
      //     {
      //         for (int i = 0; i < (int)srv.response.scene.world.collision_objects.size(); ++i)
      //         {
      //             if (srv.response.scene.world.collision_objects[i].id == "transport_box")
      //                 object_in_world = true;
      //         }
      //     }
      // }

      // Rows of the matrix are represented by the vector of  
      // AllowedCollisionEntry[] entry_values (moveit_msgs/AllowedCollisionMatrix) 
      // and the columns are represented by the boolean vector  
      // enabled[] (moveit_msgs/AllowedCollisionEntry) of each of the entry_values
      scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

      if(!client_get_scene_.call(scene_srv))
      {
          ROS_WARN("Failed to call service /get_planning_scene");
      }
      else
      {
          ROS_INFO_STREAM("Initial scene!");
          currentScene = scene_srv.response.scene;
          moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

          // ROS_INFO("size of acm_entry_names before %d", currentACM.entry_names.size());
          // ROS_INFO("size of acm_entry_values before %d", currentACM.entry_values.size());
          // ROS_INFO("size of acm_entry_values[0].entries before %d", currentACM.entry_values[0].enabled.size());

          for(size_t i=0; i<3;++i)
          {
            std::string object_id;
            object_id = "target"+ std::to_string (i);

            currentACM.entry_names.push_back(object_id);
            moveit_msgs::AllowedCollisionEntry entry;
            entry.enabled.resize(currentACM.entry_names.size());

            for(int i = 0; i < entry.enabled.size(); i++)
                entry.enabled[i] = true;
            
            //add new row to allowed collsion matrix
            currentACM.entry_values.push_back(entry);

            for(int i = 0; i < currentACM.entry_values.size(); i++)
            {
                //extend the last column of the matrix
                currentACM.entry_values[i].enabled.push_back(true);
            }

            newSceneDiff.is_diff = true;
            newSceneDiff.allowed_collision_matrix = currentACM;

            planning_scene_diff_publisher_.publish(newSceneDiff);
          }
       }

      if(!client_get_scene_.call(scene_srv))
      {
        ROS_WARN("Failed to call service /get_planning_scene");
      }
      else
      {
          ROS_INFO_STREAM("Modified scene!");
          currentScene = scene_srv.response.scene;
          moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

          // ROS_INFO("size of acm_entry_names after %d", currentACM.entry_names.size());
          // ROS_INFO("size of acm_entry_values after %d", currentACM.entry_values.size());
          // ROS_INFO("size of acm_entry_values[0].entries after %d", currentACM.entry_values[0].enabled.size());
      }



      // co.header.stamp = ros::Time::now();
      // co.header.frame_id = "/world";
      // co.id = "object_1";
      // co.primitives.resize(1);
      // co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
      // co.primitives[0].dimensions.resize(3);
      // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;
      // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
      // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;

      // shape_msgs::SolidPrimitive primitive;
      // primitive.type = primitive.BOX;
      // primitive.dimensions.resize(3);
      // primitive.dimensions[0] = 0.4;
      // primitive.dimensions[1] = 0.1;
      // primitive.dimensions[2] = 0.4;

      // // co.primitive_poses.resize(1);
      // // co.primitive_poses[0].position.x = -0.5437262654304504;
      // // co.primitive_poses[0].position.y = 0.16318392753601074;
      // // co.primitive_poses[0].position.z = 1.6496713161468506;
      // // co.primitive_poses[0].orientation.w = 1.0;

      // geometry_msgs::Pose box_pose;
      // box_pose.orientation.w = 1.0;
      // box_pose.position.x = 0.4;
      // box_pose.position.y = -0.2;
      // box_pose.position.z = 1.0;

      // co.primitives.push_back(primitive);
      // co.primitive_poses.push_back(box_pose);
      // co.operation = co.ADD;

      // std::vector<moveit_msgs::CollisionObject> collision_objects;
      // collision_objects.push_back(co);

      // // aco.object = co;
      // // aco.link_name = "EEF_Link";

      // // add_collision_object();
      // ROS_INFO("Adding collision object.");
      // // co.operation = moveit_msgs::CollisionObject::ADD;
      // pub_co.publish(co);
      // planning_scene_interface.addCollisionObjects(collision_objects);

      // ros::WallDuration(5.0).sleep();
}

PlanningScene::~PlanningScene(){}

bool PlanningScene::SetupCollisionObject(const std::string &object_id,
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
  pub_co.publish(collision_object);

  planning_scene_add.world.collision_objects.push_back(collision_object);
  planning_scene_add.is_diff = true;
  planning_scene_diff_publisher_.publish(planning_scene_add);

  ros::Duration(1.0).sleep();
  return true;
}

tf2::Quaternion PlanningScene::RPYToQuaternion(float R, float P, float Y){
  tf2::Matrix3x3 mat;
  mat.setEulerYPR(Y,P,R);

  tf2::Quaternion quat;
  mat.getRotation(quat);

  return quat;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "planning_scene_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  PlanningScene planning_scene(nh);
  return 0;
}