#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//collsiion

 moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();

    collision_object.id = "box1";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.5;
    primitive.dimensions[1] = 0.8;
    primitive.dimensions[2] = 1.0;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.applyCollisionObjects(collision_objects);

     moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group_interface_arm.getPlanningFrame();

    collision_object2.id = "box2";

    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 1.5;
    primitive2.dimensions[1] = 0.8;
    primitive2.dimensions[2] = 1.0;

    geometry_msgs::Pose box_pose2;
    box_pose2.orientation.w = 1.0;
    box_pose2.position.x = 0.0;
    box_pose2.position.y = 1.0;
    box_pose2.position.z = -0.5;

    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(box_pose2);
    collision_object2.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects2;
    collision_objects2.push_back(collision_object2);

    planning_scene_interface.applyCollisionObjects(collision_objects2);

moveit_msgs::CollisionObject collision_object3;
collision_object3.header.frame_id = move_group_interface_arm.getPlanningFrame();
collision_object3.id = "cylinder";

shape_msgs::SolidPrimitive primitive3;
primitive3.type = primitive3.CYLINDER;
primitive3.dimensions.resize(2);
primitive3.dimensions[primitive3.CYLINDER_HEIGHT] = 0.23;
primitive3.dimensions[primitive3.CYLINDER_RADIUS] = 0.03;

geometry_msgs::Pose cylinder_pose;
cylinder_pose.orientation.w = 1.0;
cylinder_pose.position.x = 0.0;
cylinder_pose.position.y = 0.77;
cylinder_pose.position.z = 0.15;

collision_object3.primitives.push_back(primitive3);
collision_object3.primitive_poses.push_back(cylinder_pose);
collision_object3.operation = collision_object3.ADD;

std::vector<moveit_msgs::CollisionObject> collision_objects3;
collision_objects3.push_back(collision_object3);

planning_scene_interface.applyCollisionObjects(collision_objects3);


    ROS_INFO_NAMED("tutorial", "Add an object into the world");

    ros::Duration(2.0).sleep(); 



    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("stretch"));
    
    bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 stretched (pose goal) %s", success ? "" : "stretch FAILED");

    move_group_interface_arm.move();
ros::WallDuration(5.0).sleep();

    // 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
 
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");

    geometry_msgs::Pose target_pose1;
  
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = -0.16;
    target_pose1.position.y = 0.7700113;
    target_pose1.position.z = 0.25;//0.138470;
    double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;
target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

move_group_interface_arm.setPoseTarget(target_pose1);

//    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 placement(pose goal) %s", success ? "" : "ee placement FAILED");

    move_group_interface_arm.move();
    move_group_interface_arm.move();
ros::Duration(5.0).sleep(); // Add a 2-second delay before the next pose goal


ros::Duration(5.0).sleep();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

    // 3. Open the gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "open gripper %s", success ? "" : "opening gripper FAILED");

    move_group_interface_gripper.move();
ros::Duration(2.0).sleep(); 

//move closer

target_pose1.position.z -= 0.11153; // Move the TCP closer to the object
  move_group_interface_arm.setPoseTarget(target_pose1);
  success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "moving closerr (pose goal) %s", success ? "" : "moving closer FAILED");
  move_group_interface_arm.move();
  


    // 5. Close the  gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("close"));


    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Vclosing gripper pose goal) %s", success ? "" : "closing gripper FAILED");

    move_group_interface_gripper.move();
ros::Duration(2.0).sleep(); 
    // 6. Move the TCP above the plate
    
    target_pose1.position.z += 0.3;
    target_pose1.position.y += 0.2 ;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "lifting cup %s", success ? "" : "lifting cupFAILED");

    move_group_interface_arm.move();

    // 7. Lower the TCP above the plate
    target_pose1.position.z = target_pose1.position.z - 0.14;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "placing cup (pose goal) %s", success ? "" : "tplace cup fAILED");

    move_group_interface_arm.move();

    // 8. Open the gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "opening grtipper final (pose goal) %s", success ? "" : "opening gripper final FAILED");

    move_group_interface_gripper.move();
ros::Duration(2.0).sleep(); 
    ROS_INFO_NAMED("tutorial", "Remove the object from the world");
    std::vector<std::string> object_ids;
   // object_ids.push_back(collision_object.id);
    //planning_scene_interface.removeCollisionObjects(object_ids);

  ros::shutdown();
  return 0;
}
