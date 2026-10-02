// #include <memory>
// #include <thread>
// #include <rclcpp/rclcpp.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit/kinematics_base/kinematics_base.h>

// #include "gripper_client.hpp"

// #include "ur_pick_and_place_msgs/srv/movetcppos.hpp"
// #include "rbpodo_msgs/srv/movetcppos.hpp"
// #include "dual_arm_msg/srv/movetcppos.hpp"
// #include "dual_arm_msg/srv/movetcppos_rb3.hpp"
// #include "dual_arm_msg/srv/movetcppos_ur3.hpp"

// #include <Eigen/Geometry>

// #define PI M_PI

// class MoveService : public rclcpp::Node
// {
// public:
//     MoveService() : Node("move_service")
//     {
//         RCLCPP_INFO(this->get_logger(), "Starting move_to_home_service...");

//         gripper_node_ = gripper_client::create_gripper_client_node("main_node");
//         gripper_client_ = gripper_client::create_gripper_service_client(gripper_node_, "gripper_service");

//         // MoveIt node options
//         rclcpp::NodeOptions node_options;
//         node_options.automatically_declare_parameters_from_overrides(true);

//         move_group_node_rb3 = rclcpp::Node::make_shared("rb3_arm", node_options);
//         executor_rb3.add_node(move_group_node_rb3);
//         spinner_rb3 = std::thread([this]() { executor_rb3.spin(); });
//         const std::string PLANNING_GROUP_RB3= "mainpulation";
//         move_group_arm_rb3 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//             move_group_node_rb3, PLANNING_GROUP_RB3);
//         joint_model_group_rb3 = move_group_arm_rb3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_RB3);
//         // [CHANGED] setWaitForMotionExecution 제거

//         move_group_node_ur3e = rclcpp::Node::make_shared("ur3e_arm", node_options);
//         executor_ur3e.add_node(move_group_node_ur3e);
//         spinner_ur3e = std::thread([this]() { executor_ur3e.spin(); });
//         const std::string PLANNING_GROUP_UR3e= "ur_manipulator";
//         move_group_arm_ur3e = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//             move_group_node_ur3e, PLANNING_GROUP_UR3e);
//         joint_model_group_ur3e = move_group_arm_ur3e->getCurrentState()->getJointModelGroup(PLANNING_GROUP_UR3e);
//         // [CHANGED] setWaitForMotionExecution 제거

//         move_group_node_bimanual = rclcpp::Node::make_shared("bimanual", node_options);
//         executor_bimanual.add_node(move_group_node_bimanual);
//         spinner_bimanual = std::thread([this]() { executor_bimanual.spin(); });
//         const std::string PLANNING_GROUP_bimanual= "dual_arm";
//         move_group_arm_bimanual = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//             move_group_node_bimanual, PLANNING_GROUP_bimanual);
//         joint_model_group_bimanual = move_group_arm_bimanual->getCurrentState()->getJointModelGroup(PLANNING_GROUP_bimanual);

//         // addCollisionObjects();

//         movetcp_service_bimanual_manual = this->create_service<dual_arm_msg::srv::Movetcppos>(
//             "zmk_move_to_tcppos", std::bind(&MoveService::movetcpposCallback, this,
//             std::placeholders::_1, std::placeholders::_2));

//         movetcp_service_bimanual = this->create_service<std_srvs::srv::Trigger>(
//             "zmk_move_to_pregrasp", std::bind(&MoveService::moveToPregraspCallback, this,
//             std::placeholders::_1, std::placeholders::_2));

//         home_service_ = this->create_service<std_srvs::srv::Trigger>(
//             "zmk_move_to_home",
//             std::bind(&MoveService::moveToHomeCallback, this,
//             std::placeholders::_1, std::placeholders::_2));

//         movetcp_service_rb3 = this->create_service<dual_arm_msg::srv::MovetcpposRB3>(
//             "zmk_tcppos_rb3", std::bind(&MoveService::rb3movetcpposCallback, this,
//             std::placeholders::_1, std::placeholders::_2));

//         movetcp_service_ur3e = this->create_service<dual_arm_msg::srv::MovetcpposUR3>(
//             "zmk_tcppos_ur3e", std::bind(&MoveService::ur3emovetcpposCallback, this,
//             std::placeholders::_1, std::placeholders::_2));

//         gripper_open_service_ = this->create_service<std_srvs::srv::Trigger>(
//             "zmk_girp_open", std::bind(&MoveService::gripopenCallback, this,
//             std::placeholders::_1, std::placeholders::_2));

//         gripper_close_service_ = this->create_service<std_srvs::srv::Trigger>(
//             "zmk_girp_close", std::bind(&MoveService::gripcloseCallback, this,
//             std::placeholders::_1, std::placeholders::_2));

//         moveapproch_rb3_service_ = this->create_service<std_srvs::srv::Trigger>(
//             "zmk_rb3_approch", std::bind(&MoveService::approchRB3Callback, this,
//             std::placeholders::_1, std::placeholders::_2));
//     }

//     ~MoveService()
//     {
//         executor_rb3.cancel();
//         if (spinner_rb3.joinable())
//             spinner_rb3.join();

//         executor_ur3e.cancel();
//         if (spinner_ur3e.joinable())
//             spinner_ur3e.join();

//         executor_bimanual.cancel();
//         if (spinner_bimanual.joinable())
//             spinner_bimanual.join();
//     }

// private:
//     double jump_threshold;
//     double eef_step;
//     double fraction;

//     rclcpp::Node::SharedPtr gripper_node_;
//     rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr gripper_client_;

//     rclcpp::Node::SharedPtr move_group_node_rb3;
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_rb3;
//     const moveit::core::JointModelGroup *joint_model_group_rb3;
//     rclcpp::executors::SingleThreadedExecutor executor_rb3;
//     std::thread spinner_rb3;

//     rclcpp::Node::SharedPtr move_group_node_ur3e;
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_ur3e;
//     const moveit::core::JointModelGroup *joint_model_group_ur3e;
//     rclcpp::executors::SingleThreadedExecutor executor_ur3e;
//     std::thread spinner_ur3e;

//     rclcpp::Node::SharedPtr move_group_node_bimanual;
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_bimanual;
//     const moveit::core::JointModelGroup *joint_model_group_bimanual;
//     rclcpp::executors::SingleThreadedExecutor executor_bimanual;
//     std::thread spinner_bimanual;

//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//     rclcpp::Service<dual_arm_msg::srv::Movetcppos>::SharedPtr movetcp_service_bimanual_manual;
//     rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr movetcp_service_bimanual;
//     rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
//     rclcpp::Service<dual_arm_msg::srv::MovetcpposRB3>::SharedPtr movetcp_service_rb3;
//     rclcpp::Service<dual_arm_msg::srv::MovetcpposUR3>::SharedPtr movetcp_service_ur3e;
//     rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_open_service_;
//     rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_close_service_;
//     rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr moveapproch_rb3_service_;

//     geometry_msgs::msg::PoseStamped right_target_pose;
//     geometry_msgs::msg::PoseStamped left_target_pose;

//     void addCollisionObjects()
//     {
//         std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

//         moveit_msgs::msg::CollisionObject ground;
//         ground.header.frame_id = move_group_arm_bimanual->getPlanningFrame();
//         ground.id = "ground";

//         shape_msgs::msg::Plane ground_plane;
//         ground_plane.coef = {0, 0, -1, 0};

//         geometry_msgs::msg::Pose ground_pose;
//         ground_pose.orientation.w = 1.0;
//         ground_pose.position.z = -0.01;

//         ground.planes.push_back(ground_plane);
//         ground.plane_poses.push_back(ground_pose);
//         ground.operation = ground.ADD;
//         collision_objects.push_back(ground);

//         planning_scene_interface.addCollisionObjects(collision_objects);
//         RCLCPP_INFO(this->get_logger(), "Added ground to the planning scene.");
//     }

//     void movetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::Movetcppos::Request> request,
//         std::shared_ptr<dual_arm_msg::srv::Movetcppos::Response> response)
//     {
//         // (원본 그대로: 양팔 동시 동작은 블로킹 유지)
//         // ... 기존 내용 ...
//         response->success = false;
//         response->message = "Not implemented here for brevity.";
//     }

//     void moveToPregraspCallback(
//         const std::shared_ptr<std_srvs::srv::Trigger::Request>,
//         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
//     {
//         // (원본 그대로: 블로킹 유지)
//         response->success = false;
//         response->message = "Not implemented here for brevity.";
//     }

//     void moveToHomeCallback(
//         const std::shared_ptr<std_srvs::srv::Trigger::Request>,
//         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
//     {
//         // (원본 그대로: 블로킹 유지)
//         response->success = false;
//         response->message = "Not implemented here for brevity.";
//     }

//     // -------------------- 오른팔(RB3): 비동기 실행 --------------------
//     void rb3movetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::MovetcpposRB3::Request> request,
//         std::shared_ptr<dual_arm_msg::srv::MovetcpposRB3::Response> response)
//     {
//         RCLCPP_INFO(this->get_logger(), "RB3_Planning TCP pose...");

//         std::string planning_frame = move_group_arm_rb3->getPlanningFrame();
//         tf2_ros::Buffer tf_buffer(this->get_clock());
//         tf2_ros::TransformListener tf_listener(tf_buffer);

//         move_group_arm_rb3->setStartStateToCurrentState();

//         tf2::Quaternion right_orientation;
//         right_orientation.setRPY(request->right_rx, request->right_ry, request->right_rz);
//         geometry_msgs::msg::Quaternion right_ros_orientation = tf2::toMsg(right_orientation);

//         right_target_pose.header.frame_id = "link0";
//         right_target_pose.header.stamp = this->now();
//         right_target_pose.pose.orientation = right_ros_orientation;
//         right_target_pose.pose.position.x = request->right_x;
//         right_target_pose.pose.position.y = request->right_y;
//         right_target_pose.pose.position.z = request->right_z;

//         geometry_msgs::msg::PoseStamped right_target_in_planning;
//         try {
//             right_target_in_planning = tf_buffer.transform(right_target_pose, planning_frame, tf2::durationFromSec(0.1));
//         } catch (tf2::TransformException & ex) {
//             response->success = false; response->message = "RB3 transform failed.";
//             return;
//         }

//         moveit::core::RobotState right_ik_state(*move_group_arm_rb3->getCurrentState());
//         std::vector<double> right_current_values;
//         move_group_arm_rb3->getCurrentState()->copyJointGroupPositions(joint_model_group_rb3, right_current_values);
//         right_ik_state.setJointGroupPositions(joint_model_group_rb3, right_current_values);

//         bool right_found_ik = right_ik_state.setFromIK(
//             joint_model_group_rb3, right_target_in_planning.pose, "tcp", 0.1);

//         if (!right_found_ik) {
//             response->success = false; response->message = "RB3 IK failed."; return;
//         }

//         std::vector<double> right_ik_joint_values;
//         right_ik_state.copyJointGroupPositions(joint_model_group_rb3, right_ik_joint_values);

//         std::map<std::string, double> right_arm_joint_goal;
//         const auto& right_joint_names = joint_model_group_rb3->getVariableNames();
//         for (size_t i = 0; i < right_joint_names.size(); ++i)
//             right_arm_joint_goal[right_joint_names[i]] = right_ik_joint_values[i];

//         move_group_arm_rb3->setStartStateToCurrentState();
//         move_group_arm_rb3->setJointValueTarget(right_arm_joint_goal);

//         moveit::planning_interface::MoveGroupInterface::Plan right_plan;
//         bool success = (move_group_arm_rb3->plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//         if (!success) {
//             response->success = false; response->message = "Right-arm planning failed."; return;
//         }

//         // [CHANGED] ---- 비동기 실행: 별도 스레드에서 execute() ----
//         std::thread([this, plan = right_plan]() {
//             move_group_arm_rb3->execute(plan);
//         }).detach();
//         response->success = true;
//         response->message = "Right-arm execution started (non-blocking)."; // 즉시 반환
//         // [CHANGED] ----------------------------------------------
//     }

//     // -------------------- 왼팔(UR3e): 비동기 실행 --------------------
//     void ur3emovetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::MovetcpposUR3::Request> request,
//         std::shared_ptr<dual_arm_msg::srv::MovetcpposUR3::Response> response)
//     {
//         RCLCPP_INFO(this->get_logger(), "UR3e_Planning TCP pose...");

//         std::string planning_frame = move_group_arm_ur3e->getPlanningFrame();
//         tf2_ros::Buffer tf_buffer(this->get_clock());
//         tf2_ros::TransformListener tf_listener(tf_buffer);

//         move_group_arm_ur3e->setStartStateToCurrentState();

//         Eigen::Vector3d rotvec(request->left_rx, request->left_ry, request->left_rz);
//         double angle = rotvec.norm();
//         Eigen::Vector3d axis = (angle < 1e-6) ? Eigen::Vector3d(1,0,0) : rotvec.normalized();
//         Eigen::AngleAxisd angle_axis(angle, axis);
//         Eigen::Quaterniond quat(angle_axis);
//         tf2::Quaternion left_orientation(quat.x(), quat.y(), quat.z(), quat.w());
//         geometry_msgs::msg::Quaternion left_ros_orientation = tf2::toMsg(left_orientation);

//         left_target_pose.header.frame_id = "base_link";
//         left_target_pose.header.stamp = this->now();
//         left_target_pose.pose.orientation = left_ros_orientation;
//         left_target_pose.pose.position.x = request->left_x;
//         left_target_pose.pose.position.y = request->left_y;
//         left_target_pose.pose.position.z = request->left_z;

//         geometry_msgs::msg::PoseStamped left_target_in_planning;
//         try {
//             left_target_in_planning = tf_buffer.transform(left_target_pose, planning_frame, tf2::durationFromSec(0.1));
//         } catch (tf2::TransformException & ex) {
//             response->success = false; response->message = "UR3e transform failed."; return;
//         }

//         moveit::core::RobotState left_ik_state(*move_group_arm_ur3e->getCurrentState());
//         std::vector<double> left_current_values;
//         move_group_arm_ur3e->getCurrentState()->copyJointGroupPositions(joint_model_group_ur3e, left_current_values);
//         left_ik_state.setJointGroupPositions(joint_model_group_ur3e, left_current_values);

//         bool left_found_ik = left_ik_state.setFromIK(
//             joint_model_group_ur3e, left_target_in_planning.pose, "tool0", 0.1);

//         if (!left_found_ik) {
//             response->success = false; response->message = "UR3e IK failed."; return;
//         }

//         std::vector<double> left_ik_joint_values;
//         left_ik_state.copyJointGroupPositions(joint_model_group_ur3e, left_ik_joint_values);

//         std::map<std::string, double> left_arm_joint_goal;
//         const auto& left_joint_names = joint_model_group_ur3e->getVariableNames();
//         for (size_t i = 0; i < left_joint_names.size(); ++i)
//             left_arm_joint_goal[left_joint_names[i]] = left_ik_joint_values[i];

//         move_group_arm_ur3e->setStartStateToCurrentState();
//         move_group_arm_ur3e->setJointValueTarget(left_arm_joint_goal);

//         moveit::planning_interface::MoveGroupInterface::Plan left_plan;
//         bool success = (move_group_arm_ur3e->plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//         if (!success) {
//             response->success = false; response->message = "Left-arm planning failed."; return;
//         }

//         // [CHANGED] ---- 비동기 실행: 별도 스레드에서 execute() ----
//         std::thread([this, plan = left_plan]() {
//             move_group_arm_ur3e->execute(plan);
//         }).detach();
//         std::stringstream ss;
//         ss << "Target Pose - Position(x,y,z): "
//            << request->left_x << ", " << request->left_y << ", " << request->left_z
//            << " | Orientation(x,y,z,w): "
//            << left_ros_orientation.x << ", " << left_ros_orientation.y << ", "
//            << left_ros_orientation.z << ", " << left_ros_orientation.w;
//         response->success = true;
//         response->message = std::string("Left-arm execution started (non-blocking). ") + ss.str();
//         // [CHANGED] ----------------------------------------------
//     }

//     void gripopenCallback(
//         const std::shared_ptr<std_srvs::srv::Trigger::Request>,
//         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
//     {
//         bool success = gripper_client::send_gripper_command(gripper_node_, gripper_client_, 0);
//         response->success = success;
//         response->message = success ? "Gripper open successful." : "Failed to open gripper.";
//     }

//     void gripcloseCallback(
//         const std::shared_ptr<std_srvs::srv::Trigger::Request>,
//         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
//     {
//         bool success = gripper_client::send_gripper_command(gripper_node_, gripper_client_, 255);
//         response->success = success;
//         response->message = success ? "Gripper close successful." : "Failed to close gripper.";
//     }

//     void approchRB3Callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
//         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
//     {
//         move_group_arm_rb3->setStartStateToCurrentState();

//         std::vector<geometry_msgs::msg::Pose> approach_waypoints;
//         geometry_msgs::msg::Pose pose = right_target_pose.pose;
//         pose.position.y += 0.004;
//         approach_waypoints.push_back(pose);

//         moveit_msgs::msg::RobotTrajectory trajectory_approach;
//         const double jump_threshold = 0.0;
//         const double eef_step = 0.001;

//         double fraction = move_group_arm_rb3->computeCartesianPath(
//             approach_waypoints, eef_step, jump_threshold, trajectory_approach);

//         if (fraction > 0.9) {
//             // [CHANGED] 여기서도 비동기 실행 원하면 스레드로 실행
//             std::thread([this, traj = trajectory_approach]() {
//                 move_group_arm_rb3->execute(traj);
//             }).detach();
//             response->success = true;
//             response->message = "Cartesian approach started (non-blocking).";
//         } else {
//             response->success = false;
//             response->message = "Cartesian approach failed with fraction: " + std::to_string(fraction);
//         }
//     }
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MoveService>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();
//     rclcpp::shutdown();
//     return 0;
// }


#include <memory>
#include <thread>
#include <map>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include "gripper_client.hpp"

#include "ur_pick_and_place_msgs/srv/movetcppos.hpp"
#include "rbpodo_msgs/srv/movetcppos.hpp"
#include "dual_arm_msg/srv/movetcppos.hpp"
#include "dual_arm_msg/srv/movetcppos_rb3.hpp"
#include "dual_arm_msg/srv/movetcppos_ur3.hpp"

#define PI M_PI

class MoveService : public rclcpp::Node
{
public:
  MoveService() : Node("move_service")
  {
    RCLCPP_INFO(this->get_logger(), "Starting move_to_home_service...");

    gripper_node_ = gripper_client::create_gripper_client_node("main_node");
    gripper_client_ = gripper_client::create_gripper_service_client(gripper_node_, "gripper_service");

    // MoveIt node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // ---------------- RB3 (Right) ----------------
    move_group_node_rb3 = rclcpp::Node::make_shared("rb3_arm", node_options);
    executor_rb3.add_node(move_group_node_rb3);
    spinner_rb3 = std::thread([this]() { executor_rb3.spin(); });
    const std::string PLANNING_GROUP_RB3 = "mainpulation"; // NOTE: keep user's group name
    move_group_arm_rb3 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_rb3, PLANNING_GROUP_RB3);
    joint_model_group_rb3 = move_group_arm_rb3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_RB3);

    // ---------------- UR3e (Left) ----------------
    move_group_node_ur3e = rclcpp::Node::make_shared("ur3e_arm", node_options);
    executor_ur3e.add_node(move_group_node_ur3e);
    spinner_ur3e = std::thread([this]() { executor_ur3e.spin(); });
    const std::string PLANNING_GROUP_UR3e = "ur_manipulator";
    move_group_arm_ur3e = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_ur3e, PLANNING_GROUP_UR3e);
    joint_model_group_ur3e = move_group_arm_ur3e->getCurrentState()->getJointModelGroup(PLANNING_GROUP_UR3e);

    // ---------------- Bimanual ----------------
    move_group_node_bimanual = rclcpp::Node::make_shared("bimanual", node_options);
    executor_bimanual.add_node(move_group_node_bimanual);
    spinner_bimanual = std::thread([this]() { executor_bimanual.spin(); });
    const std::string PLANNING_GROUP_bimanual = "dual_arm";
    move_group_arm_bimanual = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_bimanual, PLANNING_GROUP_bimanual);
    joint_model_group_bimanual = move_group_arm_bimanual->getCurrentState()->getJointModelGroup(PLANNING_GROUP_bimanual);

    // addCollisionObjects();

    // ---------------- Services ----------------
    movetcp_service_bimanual_manual = this->create_service<dual_arm_msg::srv::Movetcppos>(
        "zmk_move_to_tcppos",
        std::bind(&MoveService::movetcpposCallback, this, std::placeholders::_1, std::placeholders::_2));

    movetcp_service_bimanual = this->create_service<std_srvs::srv::Trigger>(
        "zmk_move_to_pregrasp",
        std::bind(&MoveService::moveToPregraspCallback, this, std::placeholders::_1, std::placeholders::_2));

    home_service_ = this->create_service<std_srvs::srv::Trigger>(
        "zmk_move_to_home",
        std::bind(&MoveService::moveToHomeCallback, this, std::placeholders::_1, std::placeholders::_2));

    movetcp_service_rb3 = this->create_service<dual_arm_msg::srv::MovetcpposRB3>(
        "zmk_tcppos_rb3",
        std::bind(&MoveService::rb3movetcpposCallback, this, std::placeholders::_1, std::placeholders::_2));

    movetcp_service_ur3e = this->create_service<dual_arm_msg::srv::MovetcpposUR3>(
        "zmk_tcppos_ur3e",
        std::bind(&MoveService::ur3emovetcpposCallback, this, std::placeholders::_1, std::placeholders::_2));

    gripper_open_service_ = this->create_service<std_srvs::srv::Trigger>(
        "zmk_girp_open",
        std::bind(&MoveService::gripopenCallback, this, std::placeholders::_1, std::placeholders::_2));

    gripper_close_service_ = this->create_service<std_srvs::srv::Trigger>(
        "zmk_girp_close",
        std::bind(&MoveService::gripcloseCallback, this, std::placeholders::_1, std::placeholders::_2));

    moveapproch_rb3_service_ = this->create_service<std_srvs::srv::Trigger>(
        "zmk_rb3_approch",
        std::bind(&MoveService::approchRB3Callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  ~MoveService()
  {
    executor_rb3.cancel();
    if (spinner_rb3.joinable()) spinner_rb3.join();

    executor_ur3e.cancel();
    if (spinner_ur3e.joinable()) spinner_ur3e.join();

    executor_bimanual.cancel();
    if (spinner_bimanual.joinable()) spinner_bimanual.join();
  }

private:
  double jump_threshold;
  double eef_step;
  double fraction;

  rclcpp::Node::SharedPtr gripper_node_;
  rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr gripper_client_;

  rclcpp::Node::SharedPtr move_group_node_rb3;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_rb3;
  const moveit::core::JointModelGroup *joint_model_group_rb3{};
  rclcpp::executors::SingleThreadedExecutor executor_rb3;
  std::thread spinner_rb3;

  rclcpp::Node::SharedPtr move_group_node_ur3e;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_ur3e;
  const moveit::core::JointModelGroup *joint_model_group_ur3e{};
  rclcpp::executors::SingleThreadedExecutor executor_ur3e;
  std::thread spinner_ur3e;

  rclcpp::Node::SharedPtr move_group_node_bimanual;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_bimanual;
  const moveit::core::JointModelGroup *joint_model_group_bimanual{};
  rclcpp::executors::SingleThreadedExecutor executor_bimanual;
  std::thread spinner_bimanual;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  rclcpp::Service<dual_arm_msg::srv::Movetcppos>::SharedPtr movetcp_service_bimanual_manual;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr movetcp_service_bimanual;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::Service<dual_arm_msg::srv::MovetcpposRB3>::SharedPtr movetcp_service_rb3;
  rclcpp::Service<dual_arm_msg::srv::MovetcpposUR3>::SharedPtr movetcp_service_ur3e;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_open_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_close_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr moveapproch_rb3_service_;

  geometry_msgs::msg::PoseStamped right_target_pose;
  geometry_msgs::msg::PoseStamped left_target_pose;

  void addCollisionObjects()
  {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    moveit_msgs::msg::CollisionObject ground;
    ground.header.frame_id = move_group_arm_bimanual->getPlanningFrame();
    ground.id = "ground";

    shape_msgs::msg::Plane ground_plane;
    ground_plane.coef = {0, 0, -1, 0};

    geometry_msgs::msg::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.z = -0.01;

    ground.planes.push_back(ground_plane);
    ground.plane_poses.push_back(ground_pose);
    ground.operation = ground.ADD;
    collision_objects.push_back(ground);

    planning_scene_interface.addCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(), "Added ground to the planning scene.");
  }

  // -------------------- Dual-arm manual TCP (비동기 실행) --------------------
  void movetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::Movetcppos::Request> request,
                          std::shared_ptr<dual_arm_msg::srv::Movetcppos::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Planning tcp pose (dual_arm IK)...");

    std::string planning_frame = move_group_arm_bimanual->getPlanningFrame();
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // ---- Left (UR3e)
    move_group_arm_ur3e->setStartStateToCurrentState();

    Eigen::Vector3d rotvec(request->left_rx, request->left_ry, request->left_rz);
    double angle = rotvec.norm();
    Eigen::Vector3d axis = (angle < 1e-6) ? Eigen::Vector3d(1, 0, 0) : rotvec.normalized();
    Eigen::AngleAxisd angle_axis(angle, axis);
    Eigen::Quaterniond quat(angle_axis);
    tf2::Quaternion left_orientation(quat.x(), quat.y(), quat.z(), quat.w());
    geometry_msgs::msg::Quaternion left_ros_orientation = tf2::toMsg(left_orientation);

    left_target_pose.header.frame_id = "base_link";
    left_target_pose.header.stamp = this->now();
    left_target_pose.pose.orientation = left_ros_orientation;
    left_target_pose.pose.position.x = request->left_x;
    left_target_pose.pose.position.y = request->left_y;
    left_target_pose.pose.position.z = request->left_z;

    geometry_msgs::msg::PoseStamped left_target_in_planning;
    try {
      left_target_in_planning = tf_buffer.transform(left_target_pose, planning_frame, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
      response->success = false; response->message = std::string("UR3e transform failed: ") + ex.what(); return;
    }

    moveit::core::RobotState left_ik_state(*move_group_arm_ur3e->getCurrentState());
    std::vector<double> left_current_values;
    move_group_arm_ur3e->getCurrentState()->copyJointGroupPositions(joint_model_group_ur3e, left_current_values);
    left_ik_state.setJointGroupPositions(joint_model_group_ur3e, left_current_values);

    bool left_found_ik = left_ik_state.setFromIK(joint_model_group_ur3e, left_target_in_planning.pose, "tool0", 0.1);
    if (!left_found_ik) { response->success = false; response->message = "UR3e IK failed."; return; }

    std::vector<double> left_ik_joint_values;
    left_ik_state.copyJointGroupPositions(joint_model_group_ur3e, left_ik_joint_values);

    // ---- Right (RB3)
    move_group_arm_rb3->setStartStateToCurrentState();

    tf2::Quaternion right_orientation; right_orientation.setRPY(request->right_rx, request->right_ry, request->right_rz);
    geometry_msgs::msg::Quaternion right_ros_orientation = tf2::toMsg(right_orientation);

    right_target_pose.header.frame_id = "link0";
    right_target_pose.header.stamp = this->now();
    right_target_pose.pose.orientation = right_ros_orientation;
    right_target_pose.pose.position.x = request->right_x;
    right_target_pose.pose.position.y = request->right_y;
    right_target_pose.pose.position.z = request->right_z;

    geometry_msgs::msg::PoseStamped right_target_in_planning;
    try {
      right_target_in_planning = tf_buffer.transform(right_target_pose, planning_frame, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
      response->success = false; response->message = std::string("RB3 transform failed: ") + ex.what(); return;
    }

    moveit::core::RobotState right_ik_state(*move_group_arm_rb3->getCurrentState());
    std::vector<double> right_current_values;
    move_group_arm_rb3->getCurrentState()->copyJointGroupPositions(joint_model_group_rb3, right_current_values);
    right_ik_state.setJointGroupPositions(joint_model_group_rb3, right_current_values);

    bool right_found_ik = right_ik_state.setFromIK(joint_model_group_rb3, right_target_in_planning.pose, "tcp", 0.1);
    if (!right_found_ik) { response->success = false; response->message = "RB3 IK failed."; return; }

    std::vector<double> right_ik_joint_values;
    right_ik_state.copyJointGroupPositions(joint_model_group_rb3, right_ik_joint_values);

    // ---- Build dual-arm goal
    std::map<std::string, double> dual_arm_joint_goal;
    const auto &left_joint_names  = joint_model_group_ur3e->getVariableNames();
    const auto &right_joint_names = joint_model_group_rb3->getVariableNames();

    for (size_t i = 0; i < left_joint_names.size(); ++i)
      dual_arm_joint_goal[left_joint_names[i]] = left_ik_joint_values[i];
    for (size_t i = 0; i < right_joint_names.size(); ++i)
      dual_arm_joint_goal[right_joint_names[i]] = right_ik_joint_values[i];

    move_group_arm_bimanual->setStartStateToCurrentState();
    move_group_arm_bimanual->setJointValueTarget(dual_arm_joint_goal);

    moveit::planning_interface::MoveGroupInterface::Plan bimanual_plan;
    bool success = (move_group_arm_bimanual->plan(bimanual_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) { response->success = false; response->message = "Dual-arm planning failed."; return; }

    // ---- 비동기 실행 ----
    std::thread([this, plan = bimanual_plan]() {
      move_group_arm_bimanual->execute(plan);
    }).detach();

    response->success = true;
    response->message = "Dual-arm execution started (non-blocking).";
  }

  // -------------------- Pregrasp (비동기 실행) --------------------
  void moveToPregraspCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Planning pre-grasp pose (dual_arm IK)...");

    std::string planning_frame = move_group_arm_bimanual->getPlanningFrame();
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Common orientation
    tf2::Quaternion orientation; orientation.setRPY(0, -PI, 0);
    geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);

    // Left target
    move_group_arm_ur3e->setStartStateToCurrentState();
    left_target_pose.header.frame_id = "base_link";
    left_target_pose.header.stamp = this->now();
    left_target_pose.pose.orientation = ros_orientation;
    left_target_pose.pose.position.x = 0.010;
    left_target_pose.pose.position.y = -0.410;
    left_target_pose.pose.position.z = 0.264;

    geometry_msgs::msg::PoseStamped left_target_in_planning;
    try {
      left_target_in_planning = tf_buffer.transform(left_target_pose, planning_frame, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
      response->success = false; response->message = std::string("UR3e transform failed: ") + ex.what(); return;
    }

    moveit::core::RobotState left_ik_state(*move_group_arm_ur3e->getCurrentState());
    std::vector<double> left_current_values;
    move_group_arm_ur3e->getCurrentState()->copyJointGroupPositions(joint_model_group_ur3e, left_current_values);
    left_ik_state.setJointGroupPositions(joint_model_group_ur3e, left_current_values);

    bool left_found_ik = left_ik_state.setFromIK(joint_model_group_ur3e, left_target_in_planning.pose, "tool0", 0.1);
    if (!left_found_ik) { response->success = false; response->message = "UR3e IK failed."; return; }
    std::vector<double> left_ik_joint_values; left_ik_state.copyJointGroupPositions(joint_model_group_ur3e, left_ik_joint_values);

    // Right target
    move_group_arm_rb3->setStartStateToCurrentState();
    right_target_pose.header.frame_id = "link0";
    right_target_pose.header.stamp = this->now();
    right_target_pose.pose.orientation = ros_orientation;
    right_target_pose.pose.position.x = 0.010;
    right_target_pose.pose.position.y = -0.410;
    right_target_pose.pose.position.z = 0.264;

    geometry_msgs::msg::PoseStamped right_target_in_planning;
    try {
      right_target_in_planning = tf_buffer.transform(right_target_pose, planning_frame, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
      response->success = false; response->message = std::string("RB3 transform failed: ") + ex.what(); return;
    }

    moveit::core::RobotState right_ik_state(*move_group_arm_rb3->getCurrentState());
    std::vector<double> right_current_values;
    move_group_arm_rb3->getCurrentState()->copyJointGroupPositions(joint_model_group_rb3, right_current_values);
    right_ik_state.setJointGroupPositions(joint_model_group_rb3, right_current_values);

    bool right_found_ik = right_ik_state.setFromIK(joint_model_group_rb3, right_target_in_planning.pose, "tcp", 0.1);
    if (!right_found_ik) { response->success = false; response->message = "RB3 IK failed."; return; }
    std::vector<double> right_ik_joint_values; right_ik_state.copyJointGroupPositions(joint_model_group_rb3, right_ik_joint_values);

    // Build dual goal
    std::map<std::string, double> dual_arm_joint_goal;
    const auto &left_joint_names  = joint_model_group_ur3e->getVariableNames();
    const auto &right_joint_names = joint_model_group_rb3->getVariableNames();
    for (size_t i = 0; i < left_joint_names.size(); ++i)  dual_arm_joint_goal[left_joint_names[i]]  = left_ik_joint_values[i];
    for (size_t i = 0; i < right_joint_names.size(); ++i) dual_arm_joint_goal[right_joint_names[i]] = right_ik_joint_values[i];

    move_group_arm_bimanual->setStartStateToCurrentState();
    move_group_arm_bimanual->setJointValueTarget(dual_arm_joint_goal);

    moveit::planning_interface::MoveGroupInterface::Plan bimanual_plan;
    bool success = (move_group_arm_bimanual->plan(bimanual_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) { response->success = false; response->message = "Dual-arm planning failed."; return; }

    // Async execute
    std::thread([this, plan = bimanual_plan]() {
      move_group_arm_bimanual->execute(plan);
    }).detach();

    response->success = true;
    response->message = "Dual-arm pregrasp execution started (non-blocking).";
  }

  // -------------------- Home (비동기 실행) --------------------
  void moveToHomeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Planning dual-arm home position...");

    moveit::core::RobotStatePtr current_state = move_group_arm_bimanual->getCurrentState(10);

    std::vector<double> left_joint_group_positions;
    std::vector<double> right_joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group_ur3e, left_joint_group_positions);
    current_state->copyJointGroupPositions(joint_model_group_rb3, right_joint_group_positions);

    // Example home values (from user's snippet)
    left_joint_group_positions[0] = -PI/2; left_joint_group_positions[1] = -PI/2; left_joint_group_positions[2] = 0.0;
    left_joint_group_positions[3] = -PI/2; left_joint_group_positions[4] =  PI/2; left_joint_group_positions[5] = 0.0;

    right_joint_group_positions[0] = -PI/2; right_joint_group_positions[1] = 0.0; right_joint_group_positions[2] = 0.0;
    right_joint_group_positions[3] =  PI/2; right_joint_group_positions[4] = 0.0;  right_joint_group_positions[5] = 0.0;

    std::map<std::string, double> dual_arm_joint_goal;
    const auto &left_joint_names  = joint_model_group_ur3e->getVariableNames();
    const auto &right_joint_names = joint_model_group_rb3->getVariableNames();
    for (size_t i = 0; i < left_joint_names.size(); ++i)  dual_arm_joint_goal[left_joint_names[i]]  = left_joint_group_positions[i];
    for (size_t i = 0; i < right_joint_names.size(); ++i) dual_arm_joint_goal[right_joint_names[i]] = right_joint_group_positions[i];

    move_group_arm_bimanual->setStartStateToCurrentState();
    move_group_arm_bimanual->setJointValueTarget(dual_arm_joint_goal);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm_bimanual->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) { response->success = false; response->message = "Failed to plan dual-arm to home position."; return; }

    // Async execute
    std::thread([this, plan = my_plan]() {
      move_group_arm_bimanual->execute(plan);
    }).detach();

    response->success = true;
    response->message = "Dual-arm home execution started (non-blocking).";
  }

  // -------------------- 오른팔(RB3): 비동기 실행 --------------------
  void rb3movetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::MovetcpposRB3::Request> request,
                             std::shared_ptr<dual_arm_msg::srv::MovetcpposRB3::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "RB3_Planning TCP pose...");

    std::string planning_frame = move_group_arm_rb3->getPlanningFrame();
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    move_group_arm_rb3->setStartStateToCurrentState();

    tf2::Quaternion right_orientation; right_orientation.setRPY(request->right_rx, request->right_ry, request->right_rz);
    geometry_msgs::msg::Quaternion right_ros_orientation = tf2::toMsg(right_orientation);

    right_target_pose.header.frame_id = "link0";
    right_target_pose.header.stamp = this->now();
    right_target_pose.pose.orientation = right_ros_orientation;
    right_target_pose.pose.position.x = request->right_x;
    right_target_pose.pose.position.y = request->right_y;
    right_target_pose.pose.position.z = request->right_z;

    geometry_msgs::msg::PoseStamped right_target_in_planning;
    try {
      right_target_in_planning = tf_buffer.transform(right_target_pose, planning_frame, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
      response->success = false; response->message = std::string("RB3 transform failed: ") + ex.what(); return;
    }

    moveit::core::RobotState right_ik_state(*move_group_arm_rb3->getCurrentState());
    std::vector<double> right_current_values;
    move_group_arm_rb3->getCurrentState()->copyJointGroupPositions(joint_model_group_rb3, right_current_values);
    right_ik_state.setJointGroupPositions(joint_model_group_rb3, right_current_values);

    bool right_found_ik = right_ik_state.setFromIK(joint_model_group_rb3, right_target_in_planning.pose, "tcp", 0.1);
    if (!right_found_ik) { response->success = false; response->message = "RB3 IK failed."; return; }

    std::vector<double> right_ik_joint_values; right_ik_state.copyJointGroupPositions(joint_model_group_rb3, right_ik_joint_values);

    std::map<std::string, double> right_arm_joint_goal;
    const auto &right_joint_names = joint_model_group_rb3->getVariableNames();
    for (size_t i = 0; i < right_joint_names.size(); ++i)
      right_arm_joint_goal[right_joint_names[i]] = right_ik_joint_values[i];

    move_group_arm_rb3->setStartStateToCurrentState();
    move_group_arm_rb3->setJointValueTarget(right_arm_joint_goal);

    moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    bool success = (move_group_arm_rb3->plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) { response->success = false; response->message = "Right-arm planning failed."; return; }

    // Async execute
    std::thread([this, plan = right_plan]() {
      move_group_arm_rb3->execute(plan);
    }).detach();

    response->success = true;
    response->message = "Right-arm execution started (non-blocking).";
  }

  // -------------------- 왼팔(UR3e): 비동기 실행 --------------------
  void ur3emovetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::MovetcpposUR3::Request> request,
                              std::shared_ptr<dual_arm_msg::srv::MovetcpposUR3::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "UR3e_Planning TCP pose...");

    std::string planning_frame = move_group_arm_ur3e->getPlanningFrame();
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    move_group_arm_ur3e->setStartStateToCurrentState();

    // RotVec -> Quaternion
    Eigen::Vector3d rotvec(request->left_rx, request->left_ry, request->left_rz);
    double angle = rotvec.norm();
    Eigen::Vector3d axis = (angle < 1e-6) ? Eigen::Vector3d(1, 0, 0) : rotvec.normalized();
    Eigen::AngleAxisd angle_axis(angle, axis);
    Eigen::Quaterniond quat(angle_axis);
    tf2::Quaternion left_orientation(quat.x(), quat.y(), quat.z(), quat.w());
    geometry_msgs::msg::Quaternion left_ros_orientation = tf2::toMsg(left_orientation);

    left_target_pose.header.frame_id = "base_link";
    left_target_pose.header.stamp = this->now();
    left_target_pose.pose.orientation = left_ros_orientation;
    left_target_pose.pose.position.x = request->left_x;
    left_target_pose.pose.position.y = request->left_y;
    left_target_pose.pose.position.z = request->left_z;

    geometry_msgs::msg::PoseStamped left_target_in_planning;
    try {
      left_target_in_planning = tf_buffer.transform(left_target_pose, planning_frame, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
      response->success = false; response->message = std::string("UR3e transform failed: ") + ex.what(); return;
    }

    moveit::core::RobotState left_ik_state(*move_group_arm_ur3e->getCurrentState());
    std::vector<double> left_current_values;
    move_group_arm_ur3e->getCurrentState()->copyJointGroupPositions(joint_model_group_ur3e, left_current_values);
    left_ik_state.setJointGroupPositions(joint_model_group_ur3e, left_current_values);

    bool left_found_ik = left_ik_state.setFromIK(joint_model_group_ur3e, left_target_in_planning.pose, "tool0", 0.1);
    if (!left_found_ik) { response->success = false; response->message = "UR3e IK failed."; return; }

    std::vector<double> left_ik_joint_values; left_ik_state.copyJointGroupPositions(joint_model_group_ur3e, left_ik_joint_values);

    std::map<std::string, double> left_arm_joint_goal;
    const auto &left_joint_names = joint_model_group_ur3e->getVariableNames();
    for (size_t i = 0; i < left_joint_names.size(); ++i)
      left_arm_joint_goal[left_joint_names[i]] = left_ik_joint_values[i];

    move_group_arm_ur3e->setStartStateToCurrentState();
    move_group_arm_ur3e->setJointValueTarget(left_arm_joint_goal);

    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    bool success = (move_group_arm_ur3e->plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) { response->success = false; response->message = "Left-arm planning failed."; return; }

    // Async execute
    std::thread([this, plan = left_plan]() {
      move_group_arm_ur3e->execute(plan);
    }).detach();

    std::stringstream ss;
    ss << "Left execution started (non-blocking). Target Pose - Position(x,y,z): "
       << request->left_x << ", " << request->left_y << ", " << request->left_z
       << " | Orientation(x,y,z,w): "
       << left_ros_orientation.x << ", " << left_ros_orientation.y << ", "
       << left_ros_orientation.z << ", " << left_ros_orientation.w;

    response->success = true;
    response->message = ss.str();
  }

  // -------------------- RB3 Approach (비동기 실행) --------------------
  void approchRB3Callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    move_group_arm_rb3->setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    geometry_msgs::msg::Pose pose = right_target_pose.pose;
    pose.position.y += 0.004;
    approach_waypoints.push_back(pose);

    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;

    double fraction = move_group_arm_rb3->computeCartesianPath(
        approach_waypoints, eef_step, jump_threshold, trajectory_approach);

    if (fraction > 0.9) {
      std::thread([this, traj = trajectory_approach]() {
        move_group_arm_rb3->execute(traj);
      }).detach();
      response->success = true;
      response->message = "Cartesian approach started (non-blocking).";
    } else {
      response->success = false;
      response->message = std::string("Cartesian approach failed with fraction: ") + std::to_string(fraction);
    }
  }

  // -------------------- Gripper --------------------
  void gripopenCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    bool success = gripper_client::send_gripper_command(gripper_node_, gripper_client_, 0);
    response->success = success;
    response->message = success ? "Gripper open successful." : "Failed to open gripper.";
  }

  void gripcloseCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    bool success = gripper_client::send_gripper_command(gripper_node_, gripper_client_, 255);
    response->success = success;
    response->message = success ? "Gripper close successful." : "Failed to close gripper.";
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
