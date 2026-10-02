#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/robot_state.h>

#include "gripper_client.hpp"

#define PI 3.141592

int main(int argc, char * argv[])
{
  // Create a ROS logger
  auto const LOGGER = rclcpp::get_logger("ur_pick_and_place_moveit");

  rclcpp::init(argc, argv);

  //-------------------------------gripper--------------------------------------
  auto node = gripper_client::create_gripper_client_node("main_node");
  auto client = gripper_client::create_gripper_service_client(node, "gripper_service");
  //----------------------------------------------------------------------------

  //-------------------------------moveit node----------------------------------
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("ur_pick_and_place_moveit", node_options);
  //----------------------------------------------------------------------------

  //-----------------------------------thead------------------------------------
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  //----------------------------------------------------------------------------

  //----------------------------moveit moveit group-----------------------------
  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  //----------------------------------------------------------------------------
  //---------------------------wall / ground collision--------------------------
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm.getPlanningFrame();
  collision_object.id = "ground";

  shape_msgs::msg::Plane plane;
  plane.coef = {0, 0, 1, 0}; // Equation of plane z = 0

  geometry_msgs::msg::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.z = -0.01; // z position

  collision_object.planes.push_back(plane);
  collision_object.plane_poses.push_back(ground_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  //-----------back---------------
  moveit_msgs::msg::CollisionObject back_wall;
  back_wall.header.frame_id = "base_link";
  back_wall.id = "back_wall";

  shape_msgs::msg::Plane wall_plane;
  wall_plane.coef = {0, 1, 0, 0.15};

  geometry_msgs::msg::Pose wall_pose;
  wall_pose.orientation.w = 1.0;
  wall_pose.position.x = 0.0;
  wall_pose.position.y = 0.0;
  wall_pose.position.z = 0.0;

  back_wall.planes.push_back(wall_plane);
  back_wall.plane_poses.push_back(wall_pose);
  back_wall.operation = back_wall.ADD;
  collision_objects.push_back(back_wall);

  // // -------------------- 왼쪽 벽 (Left Wall) --------------------
  // moveit_msgs::msg::CollisionObject left_wall;
  // left_wall.header.frame_id = "base_link";
  // left_wall.id = "left_wall";

  // shape_msgs::msg::Plane wall_plane_left;
  // wall_plane_left.coef = {1, 0, 0, 0.30};  // x = -0.30 위치의 평면 (왼쪽)

  // geometry_msgs::msg::Pose wall_pose_left;
  // wall_pose_left.orientation.w = 1.0;
  // wall_pose_left.position.x = 0.0;
  // wall_pose_left.position.y = 0.0;
  // wall_pose_left.position.z = 0.0;

  // left_wall.planes.push_back(wall_plane_left);
  // left_wall.plane_poses.push_back(wall_pose_left);
  // left_wall.operation = left_wall.ADD;
  // collision_objects.push_back(left_wall);

  // // -------------------- 오른쪽 벽 (Right Wall) --------------------
  // moveit_msgs::msg::CollisionObject right_wall;
  // right_wall.header.frame_id = "base_link";
  // right_wall.id = "right_wall";

  // shape_msgs::msg::Plane wall_plane_right;
  // wall_plane_right.coef = {-1, 0, 0, 0.30};  // x = +0.30 위치의 평면 (오른쪽)

  // geometry_msgs::msg::Pose wall_pose_right;
  // wall_pose_right.orientation.w = 1.0;
  // wall_pose_right.position.x = 0.0;
  // wall_pose_right.position.y = 0.0;
  // wall_pose_right.position.z = 0.0;

  // right_wall.planes.push_back(wall_plane_right);
  // right_wall.plane_poses.push_back(wall_pose_right);
  // right_wall.operation = right_wall.ADD;
  // collision_objects.push_back(right_wall);

  planning_scene_interface.addCollisionObjects(collision_objects);

  //---------------------------get currnet position--------------------------
  moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10);
  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);

  RCLCPP_INFO(LOGGER, "--------------Pre-grasp open Position-----------------");
  gripper_client::send_gripper_command(node, client, 0);  

  //------------------move to home position------------------------------------------
  RCLCPP_INFO(LOGGER, "-------------------Going Home-position-----------------");
  move_group_arm.setStartStateToCurrentState();

  joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -PI/2; // Shoulder Lift
  joint_group_positions_arm[2] = 0.00;  // Elbow
  joint_group_positions_arm[3] = -PI/2; // Wrist 1
  joint_group_positions_arm[4] = 0.00; // Wrist 2
  joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm1;
  // bool success = (move_group_arm.plan(my_plan_arm1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  bool success = (move_group_arm.plan(my_plan_arm1) == moveit::core::MoveItErrorCode::SUCCESS);
  // move_group_arm.execute(my_plan_arm1);

  moveit::core::MoveItErrorCode exec_result = move_group_arm.execute(my_plan_arm1);
  // RCLCPP_INFO(LOGGER, "#######exec_result: %d $$$$$$$$$$", exec_result.val);
  if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
    std::cout << "Motion completed successfully!" << std::endl;
    RCLCPP_INFO(LOGGER, "#######exec_result: %d $$$$$$$$$$", exec_result.val);
  } else {
      std::cout << "Execution failed." << std::endl;
  }

  rclcpp::sleep_for(std::chrono::seconds(1));


  // moveit_msgs::msg::CollisionObject front_wall;
  // front_wall.header.frame_id = "ground";
  // front_wall.id = "front_wall";
  // shape_msgs::msg::SolidPrimitive fnt_wall_box;
  // fnt_wall_box.type = fnt_wall_box.BOX;
  // fnt_wall_box.dimensions = {0.3, 0.05, 0.1};  // 얇은 벽: 두께, 폭, 높이
  // geometry_msgs::msg::Pose fnt_wall_pose;
  // fnt_wall_pose.orientation.w = 1.0;
  // fnt_wall_pose.position.y = -0.3;  // 로봇 뒤쪽
  // fnt_wall_pose.position.z = 0.15;   // 벽의 중앙 높이
  // front_wall.primitives.push_back(fnt_wall_box);
  // front_wall.primitive_poses.push_back(fnt_wall_pose);
  // front_wall.operation = front_wall.ADD;
  // std::vector<moveit_msgs::msg::CollisionObject> new_objects;
  // new_objects.push_back(front_wall);
  // planning_scene_interface.addCollisionObjects(new_objects);
  // RCLCPP_INFO(LOGGER, "박스 추가 완료!");


  //------------------move to pregrasp position------------------------------------------
  RCLCPP_INFO(LOGGER, "-----------------Pregrasp Position---------------------");
  move_group_arm.setStartStateToCurrentState();
  tf2::Quaternion orientation;
  orientation.setRPY(0, -PI, 0); // Set Rotation Roll Pitch Yaw

  geometry_msgs::msg::Quaternion ros_orientation;
  ros_orientation = tf2::toMsg(orientation);  // tf2 Quaternion -> ROS msg

  // Assign Quaternion
  std::vector<geometry_msgs::msg::Pose> approach_waypoints0;
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation = ros_orientation;
  target_pose1.position.x = 0.010;
  target_pose1.position.y = -0.410;
  target_pose1.position.z = 0.264;

  move_group_arm.setPoseTarget(target_pose1);
  // success = (move_group_arm.plan(my_plan_arm1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  success = (move_group_arm.plan(my_plan_arm1) == moveit::core::MoveItErrorCode::SUCCESS);
  if(success)
  {
    RCLCPP_INFO(LOGGER, "Planning success");
    // move_group_arm.execute(my_plan_arm1);
    exec_result = move_group_arm.execute(my_plan_arm1);
    // RCLCPP_INFO(LOGGER, "#######exec_result: %d $$$$$$$$$$", exec_result.val);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      std::cout << "Motion completed successfully!" << std::endl;
      RCLCPP_INFO(LOGGER, "#######exec_result: %d $$$$$$$$$$", exec_result.val);
    } else {
        std::cout << "Execution failed." << std::endl;
    }
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "**********Planning failed!***********");
  }

	const double jump_threshold = 0.0;
  const double eef_step = 0.01;
	double fraction;

  // target_pose1.orientation = ros_orientation;
  // // target_pose1.position.x = 0.010;
  // // target_pose1.position.y = -0.410;
  // // target_pose1.position.z = 0.264;
  // approach_waypoints0.push_back(target_pose1);

  // moveit_msgs::msg::RobotTrajectory trajectory_approach0;

  // fraction = move_group_arm.computeCartesianPath(approach_waypoints0, eef_step, jump_threshold, trajectory_approach0);
  // move_group_arm.execute(trajectory_approach0);

  rclcpp::sleep_for(std::chrono::seconds(3));

  //------------------move to pregrasp position------------------------------------------
  RCLCPP_INFO(LOGGER, "---------------------Approach to object!----------------");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints1;
  target_pose1.position.z -= 0.04;
  approach_waypoints1.push_back(target_pose1);

  target_pose1.position.z -= 0.04;
  approach_waypoints1.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach1;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;

  fraction = move_group_arm.computeCartesianPath(
      approach_waypoints1, eef_step, jump_threshold, trajectory_approach1);
  // RCLCPP_INFO(LOGGER, "===================Computed Cartesian path with fraction: %f", fraction);

  move_group_arm.execute(trajectory_approach1);

  rclcpp::sleep_for(std::chrono::seconds(1));

  //-------------------------------gripper--------------------------------------
  RCLCPP_INFO(LOGGER, "----------------grasp close Position--------------------");
  gripper_client::send_gripper_command(node, client, 255);
  //-------------------------------gripper--------------------------------------

  //-------------------------------Retreat--------------------------------------
  RCLCPP_INFO(LOGGER, "----------------Retreat from object!-------------------");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints1;
  target_pose1.position.z += 0.04;
  retreat_waypoints1.push_back(target_pose1);

  target_pose1.position.z += 0.04;
  retreat_waypoints1.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat1;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints1, eef_step, jump_threshold, trajectory_retreat1);

  move_group_arm.execute(trajectory_retreat1);

  //----------------------------------------------------------------------------

  rclcpp::sleep_for(std::chrono::seconds(1));

  //-------------------------------carrying-------------------------------------
  RCLCPP_INFO(LOGGER, "------------------Move to Placing Point----------------");

  std::vector<geometry_msgs::msg::Pose> carry_waypoints;
  target_pose1.position.x += 0.200;
  target_pose1.position.y += 0.100;
  carry_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_carry;

  fraction = move_group_arm.computeCartesianPath(
      carry_waypoints, eef_step, jump_threshold, trajectory_carry);

  move_group_arm.execute(trajectory_carry);
  //----------------------------------------------------------------------------

  rclcpp::sleep_for(std::chrono::seconds(3));

  //-------------------------------approaching----------------------------------
  RCLCPP_INFO(LOGGER, "1-----------------Approach to object!-------------------1");
  move_group_arm.setStartStateToCurrentState();

  std::vector<geometry_msgs::msg::Pose> approach_waypoints2;
  target_pose1.position.z -= 0.04;
  approach_waypoints2.push_back(target_pose1);

  target_pose1.position.z -= 0.04;
  approach_waypoints2.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach2;

  fraction = move_group_arm.computeCartesianPath(
      approach_waypoints2, eef_step, jump_threshold, trajectory_approach2);
  RCLCPP_INFO(LOGGER, "===================Computed Cartesian path with fraction: %f", fraction);

  move_group_arm.execute(trajectory_approach2);
  //----------------------------------------------------------------------------

  rclcpp::sleep_for(std::chrono::seconds(1));

  //-------------------------------gripper--------------------------------------
  RCLCPP_INFO(LOGGER, "----------------grasp open Position--------------------");
  gripper_client::send_gripper_command(node, client, 0);
  //-------------------------------gripper--------------------------------------

  // Retreat - Lift up the object along the Cartesian Path
  RCLCPP_INFO(LOGGER, "1----------------Retreat from object!---------------------1");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints2;
  target_pose1.position.z += 0.04;
  retreat_waypoints2.push_back(target_pose1);

  target_pose1.position.z += 0.04;
  retreat_waypoints2.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat2;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints2, eef_step, jump_threshold, trajectory_retreat2);

  move_group_arm.execute(trajectory_retreat2);

  rclcpp::sleep_for(std::chrono::seconds(1));
  

  // Get back UR3e to Home State 
  RCLCPP_INFO(LOGGER, "------------------Going Home-----------------------------");

  current_state_arm = move_group_arm.getCurrentState(10);

  current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);

  move_group_arm.setStartStateToCurrentState();

  joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -PI/2; // Shoulder Lift
  joint_group_positions_arm[2] = 0.00;  // Elbow
  joint_group_positions_arm[3] = -PI/2; // Wrist 1
  joint_group_positions_arm[4] = 0.00; // Wrist 2
  joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm2;
  // success = (move_group_arm.plan(my_plan_arm2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  success = (move_group_arm.plan(my_plan_arm2) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm2);

  rclcpp::sleep_for(std::chrono::seconds(1));

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
