#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

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

// Load collision objects from the planning scene
std::vector<moveit_msgs::CollisionObject> collision_objects;
collision_objects.resize(3);

moveit_msgs::CollisionObject& table_collision = collision_objects[0];
table_collision.id = "table";
table_collision.header.frame_id = move_group_interface_arm.getPlanningFrame();
table_collision.operation = moveit_msgs::CollisionObject::ADD;
table_collision.primitive_poses.resize(1);
// Set the position and orientation of the table_collision
// Set the position and orientation of the table_collision
table_collision.primitive_poses[0].position.x = 0.0;
table_collision.primitive_poses[0].position.y = 0.0;
table_collision.primitive_poses[0].position.z = 0.0;
table_collision.primitive_poses[0].orientation.x = 0.0;
table_collision.primitive_poses[0].orientation.y = 0.0;
table_collision.primitive_poses[0].orientation.z = 0.0;
table_collision.primitive_poses[0].orientation.w = 1.0;

moveit_msgs::CollisionObject& table_0_collision = collision_objects[1];
table_0_collision.id = "table_0";
table_0_collision.header.frame_id = move_group_interface_arm.getPlanningFrame();
table_0_collision.operation = moveit_msgs::CollisionObject::ADD;
table_0_collision.primitive_poses.resize(1);
// Set the position and orientation of the table_0_collision
table_0_collision.primitive_poses[0].position.x = 0.0;
table_0_collision.primitive_poses[0].position.y = 1.0;
table_0_collision.primitive_poses[0].position.z = 0.0;
table_0_collision.primitive_poses[0].orientation.x = 0.0;
table_0_collision.primitive_poses[0].orientation.y = 0.0;
table_0_collision.primitive_poses[0].orientation.z = 0.0;
table_0_collision.primitive_poses[0].orientation.w = 1.0;


moveit_msgs::CollisionObject& beer_collision = collision_objects[2];
beer_collision.id = "beer";
beer_collision.header.frame_id = move_group_interface_arm.getPlanningFrame();
beer_collision.operation = moveit_msgs::CollisionObject::ADD;
beer_collision.primitive_poses.resize(1);
// Set the position and orientation of the beer_collision
// Set the position and orientation of the beer_collision
beer_collision.primitive_poses[0].position.x = 0.000159;
beer_collision.primitive_poses[0].position.y = 0.770123;
beer_collision.primitive_poses[0].position.z = 1.015001;
beer_collision.primitive_poses[0].orientation.x = 0.000004;
beer_collision.primitive_poses[0].orientation.y = -0.000018;
beer_collision.primitive_poses[0].orientation.z = 0.000732;
beer_collision.primitive_poses[0].orientation.w = 0.000001;

// Add collision objects to the planning scene
planning_scene_interface.addCollisionObjects(collision_objects);


ros::WallDuration(5.0).sleep();


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
    target_pose1.position.x = -0.0;
    target_pose1.position.y = 0.70;
    target_pose1.position.z = 0.338470;
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
ros::Duration(10.0).sleep(); // Add a 2-second delay before the next pose goal


ros::Duration(10.0).sleep();

     // Check joint limits
  std::vector<double> joint_values;
 move_group_interface_arm.getCurrentState()->copyJointGroupPositions(move_group_interface_arm.getCurrentState()->getRobotModel()->getJointModelGroup(PLANNING_GROUP_ARM), joint_values);
  bool within_limits = move_group_interface_arm.getCurrentState()->satisfiesBounds();
  if (!within_limits)
  {
    ROS_ERROR("Goal position is outside joint limits. Invalid goal.");
    return 0;
  }
  
  //
  
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

    // 3. Open the gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 og (pose goal) %s", success ? "" : "og FAILED");

    move_group_interface_gripper.move();
ros::Duration(2.0).sleep(); 
    // 4. Move the TCP close to the object
    /**
    target_pose1.position.z = target_pose1.position.z - 0.2;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 tcp move (pose goal) %s", success ? "" : "tcp moveFAILED");

    move_group_interface_arm.move();
**/
    // 5. Close the  gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("close"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 cg(pose goal) %s", success ? "" : "cg FAILED");

    move_group_interface_gripper.move();
ros::Duration(2.0).sleep(); 
    // 6. Move the TCP above the plate
    /**
    target_pose1.position.z = target_pose1.position.z + 0.2;
    target_pose1.position.x = target_pose1.position.x - 0.6;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 mtcp(pose goal) %s", success ? "" : "mtcpFAILED");

    move_group_interface_arm.move();

    // 7. Lower the TCP above the plate
    target_pose1.position.z = target_pose1.position.z - 0.14;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 tcplow(pose goal) %s", success ? "" : "tcplowFAILED");

    move_group_interface_arm.move();
**/
    // 8. Open the gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();
ros::Duration(2.0).sleep(); 
    ROS_INFO_NAMED("tutorial", "Remove the object from the world");
    std::vector<std::string> object_ids;
   // object_ids.push_back(collision_object.id);
    //planning_scene_interface.removeCollisionObjects(object_ids);

  ros::shutdown();
  return 0;
}
