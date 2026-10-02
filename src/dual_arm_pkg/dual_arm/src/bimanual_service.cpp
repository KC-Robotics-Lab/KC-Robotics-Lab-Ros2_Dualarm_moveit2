// #include <memory>
// #include <thread>
// #include <map>
// #include <set>
// #include <sstream>
// #include <atomic> 

// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <std_srvs/srv/empty.hpp>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit/kinematics_base/kinematics_base.h>
// #include <moveit_msgs/msg/robot_trajectory.hpp>

// #include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <control_msgs/action/follow_joint_trajectory.hpp>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>

// #include <Eigen/Geometry>

// #include "gripper_client.hpp"

// #include "ur_pick_and_place_msgs/srv/movetcppos.hpp"
// #include "rbpodo_msgs/srv/movetcppos.hpp"
// #include "dual_arm_msg/srv/movetcppos.hpp"
// #include "dual_arm_msg/srv/movetcppos_rb3.hpp"
// #include "dual_arm_msg/srv/movetcppos_ur3.hpp"

// #define PI M_PI

// class MoveService : public rclcpp::Node
// {
// public:
//   using FJT = control_msgs::action::FollowJointTrajectory;

//   MoveService() : Node("move_service")
//   {
//     RCLCPP_INFO(this->get_logger(), "Starting move_to_home_service...");

//     gripper_node_ = gripper_client::create_gripper_client_node("main_node");
//     gripper_client_ = gripper_client::create_gripper_service_client(gripper_node_, "gripper_service");

//     // MoveIt node options
//     rclcpp::NodeOptions node_options;
//     node_options.automatically_declare_parameters_from_overrides(true);

//     // ---------------- RB3 (Right) ----------------
//     move_group_node_rb3 = rclcpp::Node::make_shared("rb3_arm", node_options);
//     executor_rb3.add_node(move_group_node_rb3);
//     spinner_rb3 = std::thread([this]() { executor_rb3.spin(); });
//     const std::string PLANNING_GROUP_RB3 = "mainpulation"; // NOTE: keep user's group name
//     move_group_arm_rb3 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_rb3, PLANNING_GROUP_RB3);
//     joint_model_group_rb3 = move_group_arm_rb3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_RB3);

//     // ---------------- UR3e (Left) ----------------
//     move_group_node_ur3e = rclcpp::Node::make_shared("ur3e_arm", node_options);
//     executor_ur3e.add_node(move_group_node_ur3e);
//     spinner_ur3e = std::thread([this]() { executor_ur3e.spin(); });
//     const std::string PLANNING_GROUP_UR3e = "ur_manipulator";
//     move_group_arm_ur3e = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_ur3e, PLANNING_GROUP_UR3e);
//     joint_model_group_ur3e = move_group_arm_ur3e->getCurrentState()->getJointModelGroup(PLANNING_GROUP_UR3e);

//     // ---------------- Bimanual (plan-only; execution is direct to controllers) ----------------
//     move_group_node_bimanual = rclcpp::Node::make_shared("bimanual", node_options);
//     executor_bimanual.add_node(move_group_node_bimanual);
//     spinner_bimanual = std::thread([this]() { executor_bimanual.spin(); });
//     const std::string PLANNING_GROUP_bimanual = "dual_arm";
//     move_group_arm_bimanual = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_bimanual, PLANNING_GROUP_bimanual);
//     joint_model_group_bimanual = move_group_arm_bimanual->getCurrentState()->getJointModelGroup(PLANNING_GROUP_bimanual);

//     // ---- Action clients (controller topics MUST match your setup) ----
//     // RB3 example: "/rb3_arm/joint_trajectory_controller/follow_joint_trajectory"
//     // UR3e example: "/ur3e_arm/scaled_joint_trajectory_controller/follow_joint_trajectory"
    
//     right_fjt_topic_ = this->declare_parameter<std::string>(
//       "right_fjt_topic", "/joint_trajectory_controller/follow_joint_trajectory");
//     left_fjt_topic_  = this->declare_parameter<std::string>(
//       "left_fjt_topic",  "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    
//     right_fjt_client_ = rclcpp_action::create_client<FJT>(
//       this->get_node_base_interface(),
//       this->get_node_graph_interface(),
//       this->get_node_logging_interface(),
//       this->get_node_waitables_interface(),
//       right_fjt_topic_);

//     left_fjt_client_ = rclcpp_action::create_client<FJT>(
//       this->get_node_base_interface(),
//       this->get_node_graph_interface(),
//       this->get_node_logging_interface(),
//       this->get_node_waitables_interface(),
//       left_fjt_topic_);

//     ur3_base_frame_ = this->declare_parameter<std::string>("ur3_base_frame", "base_link");
//     rb3_base_frame_ = this->declare_parameter<std::string>("rb3_base_frame", "link0");

//     // ---------------- Services ----------------
//     movetcp_service_bimanual_manual = this->create_service<dual_arm_msg::srv::Movetcppos>(
//         "zmk_move_to_tcppos", 
//         std::bind(&MoveService::movetcpposCallback, this, std::placeholders::_1, std::placeholders::_2));
//     movetcp_service_rb3 = this->create_service<dual_arm_msg::srv::MovetcpposRB3>(
//         "zmk_tcppos_rb3",
//         std::bind(&MoveService::rb3movetcpposCallback, this, std::placeholders::_1, std::placeholders::_2));

//     movetcp_service_ur3e = this->create_service<dual_arm_msg::srv::MovetcpposUR3>(
//         "zmk_tcppos_ur3e",
//         std::bind(&MoveService::ur3emovetcpposCallback, this, std::placeholders::_1, std::placeholders::_2));

//     gripper_open_service_ = this->create_service<std_srvs::srv::Trigger>(
//         "zmk_girp_open",
//         std::bind(&MoveService::gripopenCallback, this, std::placeholders::_1, std::placeholders::_2));

//     gripper_close_service_ = this->create_service<std_srvs::srv::Trigger>(
//         "zmk_girp_close",
//         std::bind(&MoveService::gripcloseCallback, this, std::placeholders::_1, std::placeholders::_2));

//     // // 파라미터로 이름 바꿀 수 있고, 기본은 /zmk_rotblock_done
//     // const std::string rotblock_done_srv_name =
//     //     this->declare_parameter<std::string>("rotblock_done_notify_service", "/zmk_rotblock_done");

//     // rotblock_done_srv_ = this->create_service<std_srvs::srv::Empty>(
//     //   rotblock_done_srv_name,
//     //   [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
//     //         std::shared_ptr<std_srvs::srv::Empty::Response> /*resp*/)
//     //   {
//     //     const auto seq = ++rotblock_notify_count_;
//     //     last_rotblock_notify_time_ = this->now();
//     //     RCLCPP_INFO(this->get_logger(),
//     //       "[ROTBLOCK] DONE notify received (count=%lu, time=%.3f)",
//     //       static_cast<unsigned long>(seq),
//     //       last_rotblock_notify_time_.seconds());
//     //   }
//     // );
//     // RCLCPP_INFO(this->get_logger(), "ROTBLOCK done notify service ready at '%s'", rotblock_done_srv_name.c_str());

//     // // 파라미터로 이름 바꿀 수 있고, 기본은 /zmk_mainseq_done
//     // const std::string mainseq_done_srv_name =
//     //     this->declare_parameter<std::string>("mainseq_done_notify_service", "/zmk_mainseq_done");

//     // mainseq_done_srv_ = this->create_service<std_srvs::srv::Empty>(
//     //   mainseq_done_srv_name,
//     //   [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
//     //         std::shared_ptr<std_srvs::srv::Empty::Response> /*resp*/)
//     //   {
//     //     const auto seq = ++mainseq_notify_count_;
//     //     last_mainseq_notify_time_ = this->now();
//     //     RCLCPP_INFO(this->get_logger(),
//     //       "[MAINSEQ] DONE notify received (count=%lu, time=%.3f)",
//     //       static_cast<unsigned long>(seq),
//     //       last_mainseq_notify_time_.seconds());
//     //   }
//     // );
//     // RCLCPP_INFO(this->get_logger(), "MAINSEQ done notify service ready at '%s'", mainseq_done_srv_name.c_str());

//   }

//   ~MoveService()
//   {
//     executor_rb3.cancel();
//     if (spinner_rb3.joinable()) spinner_rb3.join();

//     executor_ur3e.cancel();
//     if (spinner_ur3e.joinable()) spinner_ur3e.join();

//     executor_bimanual.cancel();
//     if (spinner_bimanual.joinable()) spinner_bimanual.join();
//   }

// private:
//   // ---- helpers ----
//     static void ensureTiming(trajectory_msgs::msg::JointTrajectory &traj)
//     {
//         if (traj.points.empty()) return;

//         // 1) 이미 time_from_start가 있으면 건드리지 않음
//         bool all_zero = true;
//         for (const auto &p : traj.points) {
//             if (p.time_from_start.sec > 0 || p.time_from_start.nanosec > 0) {
//             all_zero = false;
//             break;
//             }
//         }
//         if (!all_zero) return;

//         // 2) 없으면 50Hz 기준으로 채우기 (0.02s씩 증가)
//         double acc_sec = 0.0;
//         const double step_sec = 0.02;
//         for (auto &p : traj.points) {
//             acc_sec += step_sec;
//             const int64_t ns = static_cast<int64_t>(acc_sec * 1e9);
//             p.time_from_start.sec     = static_cast<int32_t>(ns / 1000000000);
//             p.time_from_start.nanosec = static_cast<uint32_t>(ns % 1000000000);
//         }
//     }

//     void sendFJTAsync(const rclcpp_action::Client<FJT>::SharedPtr &client,  // FJT 액션 서버와 통신하는 클라이언트
//                       trajectory_msgs::msg::JointTrajectory traj,           // 실행할 JointTrajectory
//                       const rclcpp::Time &start_time,                       // trajectory 시작 시간 (ROS clock 기반)
//                       const char* label)                                    // 로그 출력 시 식별용 문자열
//     {
//       traj.header.stamp = start_time;
//       ensureTiming(traj);

//       std::thread([this, client, traj, label]() {
//         if (!client->wait_for_action_server(std::chrono::seconds(3))) {
//           RCLCPP_ERROR(this->get_logger(), "FJT server not available for %s", label);
//           return;
//         }

//         FJT::Goal goal;
//         goal.trajectory = traj;

//         using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FJT>;
//         rclcpp_action::Client<FJT>::SendGoalOptions opts;

//         // [ADDED] Goal 수락/거절 콜백
//         opts.goal_response_callback =
//           [this, label](std::shared_ptr<GoalHandleFJT> gh) {
//             if (!gh) {
//               RCLCPP_ERROR(this->get_logger(), "[%s] FJT goal REJECTED by server.", label);
//             } else {
//               RCLCPP_INFO(this->get_logger(),  "[%s] FJT goal ACCEPTED. Waiting for result...", label);
//             }
//           };

//         // [ADDED] 피드백 콜백 (과하게 찍히지 않도록 DEBUG로)
//         opts.feedback_callback =
//           [this, label](GoalHandleFJT::SharedPtr /*gh*/,
//                         const std::shared_ptr<const FJT::Feedback> fb) {
//             // 필요하면 주석 해제
//             // RCLCPP_DEBUG(this->get_logger(),
//             //              "[%s] feedback: actual=%zu desired=%zu",
//             //              label, fb->actual.positions.size(), fb->desired.positions.size());
//           };

//         // [ADDED] 결과 콜백 (여기가 '실제 로봇 동작 완료' 지점)
//         opts.result_callback =
//           [this, label](const GoalHandleFJT::WrappedResult &res) {
//             using RC = rclcpp_action::ResultCode;
//             switch (res.code) {
//               case RC::SUCCEEDED:
//                 RCLCPP_INFO(this->get_logger(), "[%s] FJT DONE (SUCCEEDED).", label);
//                 break;
//               case RC::ABORTED:
//                 RCLCPP_ERROR(this->get_logger(), "[%s] FJT DONE (ABORTED).", label);
//                 break;
//               case RC::CANCELED:
//                 RCLCPP_WARN(this->get_logger(),  "[%s] FJT DONE (CANCELED).", label);
//                 break;
//               default:
//                 RCLCPP_ERROR(this->get_logger(), "[%s] FJT DONE (UNKNOWN code=%d).",
//                             label, static_cast<int>(res.code));
//             }
//           };

//         client->async_send_goal(goal, opts);
//       }).detach();
//     }

// private:
//   double jump_threshold{};
//   double eef_step{};
//   double fraction{};

//   rclcpp::Node::SharedPtr gripper_node_;
//   rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr gripper_client_;

//   rclcpp::Node::SharedPtr move_group_node_rb3;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_rb3;
//   const moveit::core::JointModelGroup *joint_model_group_rb3{};
//   rclcpp::executors::SingleThreadedExecutor executor_rb3;
//   std::thread spinner_rb3;

//   rclcpp::Node::SharedPtr move_group_node_ur3e;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_ur3e;
//   const moveit::core::JointModelGroup *joint_model_group_ur3e{};
//   rclcpp::executors::SingleThreadedExecutor executor_ur3e;
//   std::thread spinner_ur3e;

//   rclcpp::Node::SharedPtr move_group_node_bimanual;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_bimanual;
//   const moveit::core::JointModelGroup *joint_model_group_bimanual{};
//   rclcpp::executors::SingleThreadedExecutor executor_bimanual;
//   std::thread spinner_bimanual;

//   // Direct controller clients
//   rclcpp_action::Client<FJT>::SharedPtr right_fjt_client_;
//   rclcpp_action::Client<FJT>::SharedPtr left_fjt_client_;

//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//   rclcpp::Service<dual_arm_msg::srv::Movetcppos>::SharedPtr movetcp_service_bimanual_manual;
//   rclcpp::Service<dual_arm_msg::srv::MovetcpposRB3>::SharedPtr movetcp_service_rb3;
//   rclcpp::Service<dual_arm_msg::srv::MovetcpposUR3>::SharedPtr movetcp_service_ur3e;
//   rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_open_service_;
//   rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_close_service_;

//   geometry_msgs::msg::PoseStamped right_target_pose;
//   geometry_msgs::msg::PoseStamped left_target_pose;

//   // 클래스 멤버(프라이빗) 추가
//   std::string right_fjt_topic_;
//   std::string left_fjt_topic_;

//   std::string ur3_base_frame_;
//   std::string rb3_base_frame_;

//   // // ---- ROTBLOCK done notify server ----
//   // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr rotblock_done_srv_;
//   // std::atomic<uint64_t> rotblock_notify_count_{0};
//   // rclcpp::Time last_rotblock_notify_time_;

//   // // ---- MAINSEQ done notify server ----
//   // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mainseq_done_srv_;
//   // std::atomic<uint64_t> mainseq_notify_count_{0};
//   // rclcpp::Time last_mainseq_notify_time_;

//   void movetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::Movetcppos::Request> request,
//       std::shared_ptr<dual_arm_msg::srv::Movetcppos::Response> response)
//   {
//       RCLCPP_INFO(this->get_logger(), "Planning tcp pose (dual_arm IK)...");

//       move_group_arm_bimanual->setMaxVelocityScalingFactor(1.0);
//       move_group_arm_bimanual->setMaxAccelerationScalingFactor(1.0);

//       // Get bimanual planning frame
//       std::string planning_frame = move_group_arm_bimanual->getPlanningFrame();
//       RCLCPP_INFO(this->get_logger(), "Dual-arm planning frame: %s", planning_frame.c_str());

//       // 준비: tf2 Buffer + Listener
//       tf2_ros::Buffer tf_buffer(this->get_clock());
//       tf2_ros::TransformListener tf_listener(tf_buffer);

//       // Step 1: UR3e IK
//       move_group_arm_ur3e->setStartStateToCurrentState();

//       // UR 회전벡터 (Rx, Ry, Rz) → Quaternion 변환
//       Eigen::Vector3d rotvec(request->left_rx, request->left_ry, request->left_rz);
//       double angle = rotvec.norm();
//       Eigen::Vector3d axis = (angle < 1e-6) ? Eigen::Vector3d(1, 0, 0) : rotvec.normalized();

//       Eigen::AngleAxisd angle_axis(angle, axis);
//       Eigen::Quaterniond quat(angle_axis);

//       // tf2 Quaternion으로 변환
//       tf2::Quaternion left_orientation(quat.x(), quat.y(), quat.z(), quat.w());

//       // geometry_msgs 쿼터니언 메시지로 변환
//       geometry_msgs::msg::Quaternion left_ros_orientation = tf2::toMsg(left_orientation);

//       // PoseStamped 생성 및 세팅
//       // geometry_msgs::msg::PoseStamped left_target_pose;
//       left_target_pose.header.frame_id = ur3_base_frame_;
//       left_target_pose.header.stamp = this->now();
//       left_target_pose.pose.orientation = left_ros_orientation;
//       left_target_pose.pose.position.x = request->left_x;
//       left_target_pose.pose.position.y = request->left_y;
//       left_target_pose.pose.position.z = request->left_z;

//       geometry_msgs::msg::PoseStamped left_target_in_planning;
//       try {
//           left_target_in_planning = tf_buffer.transform(left_target_pose, planning_frame, tf2::durationFromSec(0.1));
//       } catch (tf2::TransformException &ex) {
//           RCLCPP_ERROR(this->get_logger(), "Transform error for UR3e: %s", ex.what());
//           response->success = false;
//           response->message = "UR3e transform failed.";
//           return;
//       }

//       moveit::core::RobotState left_ik_state(*move_group_arm_ur3e->getCurrentState());
//       std::vector<double> left_current_values;
//       move_group_arm_ur3e->getCurrentState()->copyJointGroupPositions(joint_model_group_ur3e, left_current_values);
//       left_ik_state.setJointGroupPositions(joint_model_group_ur3e, left_current_values);

//       bool left_found_ik = left_ik_state.setFromIK(
//           joint_model_group_ur3e,
//           left_target_in_planning.pose,
//           "tool0",
//           0.1
//       );

//       if (!left_found_ik)
//       {
//           RCLCPP_ERROR(this->get_logger(), "UR3e IK solution not found.");
//           response->success = false;
//           response->message = "UR3e IK failed.";
//           return;
//       }

//       std::vector<double> left_ik_joint_values;
//       left_ik_state.copyJointGroupPositions(joint_model_group_ur3e, left_ik_joint_values);

//       // Step 2: RB3 IK
//       move_group_arm_rb3->setStartStateToCurrentState();

//       tf2::Quaternion right_orientation;
//       right_orientation.setRPY(request->right_rx, request->right_ry, request->right_rz);
//       geometry_msgs::msg::Quaternion right_ros_orientation = tf2::toMsg(right_orientation);

//       // geometry_msgs::msg::PoseStamped right_target_pose;
//       right_target_pose.header.frame_id = rb3_base_frame_;
//       right_target_pose.header.stamp = this->now();
//       right_target_pose.pose.orientation = right_ros_orientation;
//       right_target_pose.pose.position.x = request->right_x;
//       right_target_pose.pose.position.y = request->right_y;
//       right_target_pose.pose.position.z = request->right_z;

//       geometry_msgs::msg::PoseStamped right_target_in_planning;
//       try {
//           right_target_in_planning = tf_buffer.transform(right_target_pose, planning_frame, tf2::durationFromSec(0.1));
//       } catch (tf2::TransformException & ex) {
//           RCLCPP_ERROR(this->get_logger(), "Transform error for RB3: %s", ex.what());
//           response->success = false;
//           response->message = "RB3 transform failed.";
//           return;
//       }

//       moveit::core::RobotState right_ik_state(*move_group_arm_rb3->getCurrentState());
//       std::vector<double> right_current_values;
//       move_group_arm_rb3->getCurrentState()->copyJointGroupPositions(joint_model_group_rb3, right_current_values);
//       right_ik_state.setJointGroupPositions(joint_model_group_rb3, right_current_values);

//       bool right_found_ik = right_ik_state.setFromIK(
//           joint_model_group_rb3,
//           right_target_in_planning.pose,
//           "tcp",
//           0.1
//       );

//       if (!right_found_ik)
//       {
//           RCLCPP_ERROR(this->get_logger(), "RB3 IK solution not found.");
//           response->success = false;
//           response->message = "RB3 IK failed.";
//           return;
//       }

//       std::vector<double> right_ik_joint_values;
//       right_ik_state.copyJointGroupPositions(joint_model_group_rb3, right_ik_joint_values);

//       // Step 3: dual_arm joint target 만들기
//       std::map<std::string, double> dual_arm_joint_goal;

//       const std::vector<std::string>& left_joint_names = joint_model_group_ur3e->getVariableNames();
//       const std::vector<std::string>& right_joint_names = joint_model_group_rb3->getVariableNames();

//       for (size_t i = 0; i < left_joint_names.size(); ++i)
//       {
//           dual_arm_joint_goal[left_joint_names[i]] = left_ik_joint_values[i];
//       }

//       for (size_t i = 0; i < right_joint_names.size(); ++i)
//       {
//           dual_arm_joint_goal[right_joint_names[i]] = right_ik_joint_values[i];
//       }

//       // Step 4: dual_arm planning + execute
//       move_group_arm_bimanual->setStartStateToCurrentState();
//       move_group_arm_bimanual->setJointValueTarget(dual_arm_joint_goal);

//       moveit::planning_interface::MoveGroupInterface::Plan bimanual_plan;
//       bool success = (move_group_arm_bimanual->plan(bimanual_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//       if (success)
//       {
//           RCLCPP_INFO(this->get_logger(), "Dual-arm plan successful, executing...");
//           move_group_arm_bimanual->execute(bimanual_plan);

//           response->success = true;
//           response->message = "Dual-arm moved to pregrasp pose successfully.";
//       }
//       else
//       {
//           RCLCPP_ERROR(this->get_logger(), "Dual-arm planning failed.");
//           response->success = false;
//           response->message = "Dual-arm planning failed.";
//       }
//   }

//   // -------------------- 오른팔(RB3): plan + FJT execute --------------------
//   void rb3movetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::MovetcpposRB3::Request> request,
//                              std::shared_ptr<dual_arm_msg::srv::MovetcpposRB3::Response> response)
//   {
//     RCLCPP_INFO(this->get_logger(), "RB3 plan + FJT exec...");

//     tf2_ros::Buffer tf_buffer(this->get_clock());
//     tf2_ros::TransformListener tf_listener(tf_buffer);

//     move_group_arm_rb3->setStartStateToCurrentState();

//     tf2::Quaternion rq; rq.setRPY(request->right_rx, request->right_ry, request->right_rz);
//     geometry_msgs::msg::Quaternion rqo = tf2::toMsg(rq);

//     right_target_pose.header.frame_id = rb3_base_frame_;
//     right_target_pose.header.stamp = this->now();
//     right_target_pose.pose.orientation = rqo;
//     right_target_pose.pose.position.x = request->right_x;
//     right_target_pose.pose.position.y = request->right_y;
//     right_target_pose.pose.position.z = request->right_z;

//     geometry_msgs::msg::PoseStamped rp;
//     try { rp = tf_buffer.transform(right_target_pose, move_group_arm_rb3->getPlanningFrame(), tf2::durationFromSec(0.1)); }
//     catch (tf2::TransformException &ex) { response->success=false; response->message=std::string("RB3 transform failed: ")+ex.what(); return; }

//     moveit::core::RobotState rs(*move_group_arm_rb3->getCurrentState());
//     std::vector<double> rv; move_group_arm_rb3->getCurrentState()->copyJointGroupPositions(joint_model_group_rb3, rv);
//     rs.setJointGroupPositions(joint_model_group_rb3, rv);
//     if (!rs.setFromIK(joint_model_group_rb3, rp.pose, "tcp", 0.1)) { response->success=false; response->message="RB3 IK failed."; return; }

//     std::vector<double> rj; rs.copyJointGroupPositions(joint_model_group_rb3, rj);
//     std::map<std::string,double> rg; const auto &rn = joint_model_group_rb3->getVariableNames(); for(size_t i=0;i<rn.size();++i) rg[rn[i]] = rj[i];
//     move_group_arm_rb3->setJointValueTarget(rg);

//     move_group_arm_rb3->setMaxVelocityScalingFactor(1.0);
//     move_group_arm_rb3->setMaxAccelerationScalingFactor(1.0);

//     moveit::planning_interface::MoveGroupInterface::Plan rplan;
//     if (move_group_arm_rb3->plan(rplan) != moveit::core::MoveItErrorCode::SUCCESS) { response->success=false; response->message="Right-arm planning failed."; return; }

//     auto start_time = this->now() + rclcpp::Duration::from_seconds(0.2);
//     sendFJTAsync(right_fjt_client_, rplan.trajectory_.joint_trajectory, start_time, "RIGHT");

//     response->success = true;
//     response->message = "Right-arm execution started via FJT (non-blocking).";
//   }

//   // -------------------- 왼팔(UR3e): plan + FJT execute --------------------
//   void ur3emovetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::MovetcpposUR3::Request> request,
//                               std::shared_ptr<dual_arm_msg::srv::MovetcpposUR3::Response> response)
//   {
//     RCLCPP_INFO(this->get_logger(), "UR3e plan + FJT exec...");

//     tf2_ros::Buffer tf_buffer(this->get_clock());
//     tf2_ros::TransformListener tf_listener(tf_buffer);

//     move_group_arm_ur3e->setStartStateToCurrentState();

//     Eigen::Vector3d rv(request->left_rx, request->left_ry, request->left_rz);
//     double angle = rv.norm();
//     Eigen::Vector3d axis = (angle < 1e-6) ? Eigen::Vector3d(1, 0, 0) : rv.normalized();
//     Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));
//     geometry_msgs::msg::Quaternion qmsg = tf2::toMsg(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));

//     left_target_pose.header.frame_id = ur3_base_frame_;
//     left_target_pose.header.stamp = this->now();
//     left_target_pose.pose.orientation = qmsg;
//     left_target_pose.pose.position.x = request->left_x;
//     left_target_pose.pose.position.y = request->left_y;
//     left_target_pose.pose.position.z = request->left_z;

//     geometry_msgs::msg::PoseStamped lp;
//     try { lp = tf_buffer.transform(left_target_pose, move_group_arm_ur3e->getPlanningFrame(), tf2::durationFromSec(0.1)); }
//     catch (tf2::TransformException &ex) { response->success=false; response->message=std::string("UR3e transform failed: ")+ex.what(); return; }

//     moveit::core::RobotState ls(*move_group_arm_ur3e->getCurrentState());
//     std::vector<double> lv; move_group_arm_ur3e->getCurrentState()->copyJointGroupPositions(joint_model_group_ur3e, lv);
//     ls.setJointGroupPositions(joint_model_group_ur3e, lv);
//     if (!ls.setFromIK(joint_model_group_ur3e, lp.pose, "tool0", 0.1)) { response->success=false; response->message="UR3e IK failed."; return; }

//     std::vector<double> lj; ls.copyJointGroupPositions(joint_model_group_ur3e, lj);
//     std::map<std::string,double> lg; const auto &ln = joint_model_group_ur3e->getVariableNames(); for(size_t i=0;i<ln.size();++i) lg[ln[i]] = lj[i];
//     move_group_arm_ur3e->setJointValueTarget(lg);

//     move_group_arm_ur3e->setMaxVelocityScalingFactor(1.0);
//     move_group_arm_ur3e->setMaxAccelerationScalingFactor(1.0);

//     moveit::planning_interface::MoveGroupInterface::Plan lplan;
//     if (move_group_arm_ur3e->plan(lplan) != moveit::core::MoveItErrorCode::SUCCESS) { response->success=false; response->message="Left-arm planning failed."; return; }

//     auto start_time = this->now() + rclcpp::Duration::from_seconds(0.2);
//     sendFJTAsync(left_fjt_client_, lplan.trajectory_.joint_trajectory, start_time, "LEFT");

//     std::stringstream ss;
//     ss << "Left execution started via FJT (non-blocking). Pos(x,y,z): "
//        << request->left_x << ", " << request->left_y << ", " << request->left_z;
//     response->success = true;
//     response->message = ss.str();
//   }

//   // -------------------- Gripper --------------------
//   // bimanual_service.cpp - 바뀐 그리퍼 서비스(즉시 ACK)
//   void gripopenCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
//                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
//   {
//     // ❶ 바로 응답(ACK) 반환
//     response->success = true;
//     response->message = "Gripper open dispatched (fire-and-forget).";

//     // ❷ 실제 하드웨어 명령은 백그라운드에서
//     std::thread([node = gripper_node_, cli = gripper_client_]() {
//       (void)gripper_client::send_gripper_command(node, cli, 0);
//     }).detach();
//   }

//   void gripcloseCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
//                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
//   {
//     response->success = true;
//     response->message = "Gripper close dispatched (fire-and-forget).";

//     std::thread([node = gripper_node_, cli = gripper_client_]() {
//       (void)gripper_client::send_gripper_command(node, cli, 255);
//     }).detach();
//   }
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<MoveService>();
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(node);
//   executor.spin();
//   rclcpp::shutdown();
//   return 0;
// }



#include <memory>
#include <thread>
#include <map>
#include <set>
#include <sstream>
#include <atomic> 

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

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
  using FJT = control_msgs::action::FollowJointTrajectory;

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

    // ---------------- Bimanual (plan-only; execution is direct to controllers) ----------------
    move_group_node_bimanual = rclcpp::Node::make_shared("bimanual", node_options);
    executor_bimanual.add_node(move_group_node_bimanual);
    spinner_bimanual = std::thread([this]() { executor_bimanual.spin(); });
    const std::string PLANNING_GROUP_bimanual = "dual_arm";
    move_group_arm_bimanual = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_bimanual, PLANNING_GROUP_bimanual);
    joint_model_group_bimanual = move_group_arm_bimanual->getCurrentState()->getJointModelGroup(PLANNING_GROUP_bimanual);

    // ---- Action clients (controller topics MUST match your setup) ----
    // RB3 example: "/rb3_arm/joint_trajectory_controller/follow_joint_trajectory"
    // UR3e example: "/ur3e_arm/scaled_joint_trajectory_controller/follow_joint_trajectory"
    
    right_fjt_topic_ = this->declare_parameter<std::string>(
      "right_fjt_topic", "/joint_trajectory_controller/follow_joint_trajectory");
    left_fjt_topic_  = this->declare_parameter<std::string>(
      "left_fjt_topic",  "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    
    right_fjt_client_ = rclcpp_action::create_client<FJT>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      right_fjt_topic_);

    left_fjt_client_ = rclcpp_action::create_client<FJT>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      left_fjt_topic_);

    ur3_base_frame_ = this->declare_parameter<std::string>("ur3_base_frame", "base_link");
    rb3_base_frame_ = this->declare_parameter<std::string>("rb3_base_frame", "link0");

    // ---------------- Services ----------------
    movetcp_service_bimanual_manual = this->create_service<dual_arm_msg::srv::Movetcppos>(
        "zmk_move_to_tcppos", 
        std::bind(&MoveService::movetcpposCallback, this, std::placeholders::_1, std::placeholders::_2));
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

    // // 파라미터로 이름 바꿀 수 있고, 기본은 /zmk_rotblock_done
    // const std::string rotblock_done_srv_name =
    //     this->declare_parameter<std::string>("rotblock_done_notify_service", "/zmk_rotblock_done");

    // rotblock_done_srv_ = this->create_service<std_srvs::srv::Empty>(
    //   rotblock_done_srv_name,
    //   [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    //         std::shared_ptr<std_srvs::srv::Empty::Response> /*resp*/)
    //   {
    //     const auto seq = ++rotblock_notify_count_;
    //     last_rotblock_notify_time_ = this->now();
    //     RCLCPP_INFO(this->get_logger(),
    //       "[ROTBLOCK] DONE notify received (count=%lu, time=%.3f)",
    //       static_cast<unsigned long>(seq),
    //       last_rotblock_notify_time_.seconds());
    //   }
    // );
    // RCLCPP_INFO(this->get_logger(), "ROTBLOCK done notify service ready at '%s'", rotblock_done_srv_name.c_str());

    // // 파라미터로 이름 바꿀 수 있고, 기본은 /zmk_mainseq_done
    // const std::string mainseq_done_srv_name =
    //     this->declare_parameter<std::string>("mainseq_done_notify_service", "/finish_mainseq");

    // mainseq_done_srv_ = this->create_service<std_srvs::srv::Empty>(
    //   mainseq_done_srv_name,
    //   [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    //         std::shared_ptr<std_srvs::srv::Empty::Response> /*resp*/)
    //   {
    //     const auto seq = ++mainseq_notify_count_;
    //     last_mainseq_notify_time_ = this->now();
    //     RCLCPP_INFO(this->get_logger(),
    //       "[MAINSEQ] DONE notify received (count=%lu, time=%.3f)",
    //       static_cast<unsigned long>(seq),
    //       last_mainseq_notify_time_.seconds());
    //   }
    // );
    // RCLCPP_INFO(this->get_logger(), "MAINSEQ done notify service ready at '%s'", mainseq_done_srv_name.c_str());

  //   // ★ ADDED: /zmk_4step_done 서비스 서버
  //   step4_done_srv_ = this->create_service<std_srvs::srv::Empty>(
  //     "/cam_check_prev",
  //     [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
  //            std::shared_ptr<std_srvs::srv::Empty::Response> /*resp*/)
  //     {
  //       const auto seq = ++step4_notify_count_;
  //       last_step4_notify_time_ = this->now();
  //       RCLCPP_INFO(this->get_logger(),
  //         "[4STEP] DONE notify received (count=%lu, time=%.3f)",
  //         static_cast<unsigned long>(seq),
  //         last_step4_notify_time_.seconds());
  //     }
  //   );
  //   RCLCPP_INFO(this->get_logger(), "4STEP done notify service ready at '/zmk_4step_done'");

  //   // ★ ADDED: /zmk_5step_done 서비스 서버
  //   step5_done_srv_ = this->create_service<std_srvs::srv::Empty>(
  //     "/cam_check",
  //     [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
  //            std::shared_ptr<std_srvs::srv::Empty::Response> /*resp*/)
  //     {
  //       const auto seq = ++step5_notify_count_;
  //       last_step5_notify_time_ = this->now();
  //       RCLCPP_INFO(this->get_logger(),
  //         "[5STEP] DONE notify received (count=%lu, time=%.3f)",
  //         static_cast<unsigned long>(seq),
  //         last_step5_notify_time_.seconds());
  //     }
  //   );
  //   RCLCPP_INFO(this->get_logger(), "5STEP done notify service ready at '/zmk_5step_done'");

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
  // ---- helpers ----
    static void ensureTiming(trajectory_msgs::msg::JointTrajectory &traj)
    {
        if (traj.points.empty()) return;

        // 1) 이미 time_from_start가 있으면 건드리지 않음
        bool all_zero = true;
        for (const auto &p : traj.points) {
            if (p.time_from_start.sec > 0 || p.time_from_start.nanosec > 0) {
            all_zero = false;
            break;
            }
        }
        if (!all_zero) return;

        // 2) 없으면 50Hz 기준으로 채우기 (0.02s씩 증가)
        double acc_sec = 0.0;
        const double step_sec = 0.02;
        for (auto &p : traj.points) {
            acc_sec += step_sec;
            const int64_t ns = static_cast<int64_t>(acc_sec * 1e9);
            p.time_from_start.sec     = static_cast<int32_t>(ns / 1000000000);
            p.time_from_start.nanosec = static_cast<uint32_t>(ns % 1000000000);
        }
    }

    void sendFJTAsync(const rclcpp_action::Client<FJT>::SharedPtr &client,  // FJT 액션 서버와 통신하는 클라이언트
                      trajectory_msgs::msg::JointTrajectory traj,           // 실행할 JointTrajectory
                      const rclcpp::Time &start_time,                       // trajectory 시작 시간 (ROS clock 기반)
                      const char* label)                                    // 로그 출력 시 식별용 문자열
    {
      traj.header.stamp = start_time;
      ensureTiming(traj);

      std::thread([this, client, traj, label]() {
        if (!client->wait_for_action_server(std::chrono::seconds(3))) {
          RCLCPP_ERROR(this->get_logger(), "FJT server not available for %s", label);
          return;
        }

        FJT::Goal goal;
        goal.trajectory = traj;

        using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FJT>;
        rclcpp_action::Client<FJT>::SendGoalOptions opts;

        // [ADDED] Goal 수락/거절 콜백
        opts.goal_response_callback =
          [this, label](std::shared_ptr<GoalHandleFJT> gh) {
            if (!gh) {
              RCLCPP_ERROR(this->get_logger(), "[%s] FJT goal REJECTED by server.", label);
            } else {
              RCLCPP_INFO(this->get_logger(),  "[%s] FJT goal ACCEPTED. Waiting for result...", label);
            }
          };

        // [ADDED] 피드백 콜백 (과하게 찍히지 않도록 DEBUG로)
        opts.feedback_callback =
          [this, label](GoalHandleFJT::SharedPtr /*gh*/,
                        const std::shared_ptr<const FJT::Feedback> fb) {
            // 필요하면 주석 해제
            // RCLCPP_DEBUG(this->get_logger(),
            //              "[%s] feedback: actual=%zu desired=%zu",
            //              label, fb->actual.positions.size(), fb->desired.positions.size());
          };

        // [ADDED] 결과 콜백 (여기가 '실제 로봇 동작 완료' 지점)
        opts.result_callback =
          [this, label](const GoalHandleFJT::WrappedResult &res) {
            using RC = rclcpp_action::ResultCode;
            switch (res.code) {
              case RC::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "[%s] FJT DONE (SUCCEEDED).", label);
                break;
              case RC::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "[%s] FJT DONE (ABORTED).", label);
                break;
              case RC::CANCELED:
                RCLCPP_WARN(this->get_logger(),  "[%s] FJT DONE (CANCELED).", label);
                break;
              default:
                RCLCPP_ERROR(this->get_logger(), "[%s] FJT DONE (UNKNOWN code=%d).",
                            label, static_cast<int>(res.code));
            }
          };

        client->async_send_goal(goal, opts);
      }).detach();
    }

private:
  double jump_threshold{};
  double eef_step{};
  double fraction{};

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

  // Direct controller clients
  rclcpp_action::Client<FJT>::SharedPtr right_fjt_client_;
  rclcpp_action::Client<FJT>::SharedPtr left_fjt_client_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  rclcpp::Service<dual_arm_msg::srv::Movetcppos>::SharedPtr movetcp_service_bimanual_manual;
  rclcpp::Service<dual_arm_msg::srv::MovetcpposRB3>::SharedPtr movetcp_service_rb3;
  rclcpp::Service<dual_arm_msg::srv::MovetcpposUR3>::SharedPtr movetcp_service_ur3e;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_open_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_close_service_;

  geometry_msgs::msg::PoseStamped right_target_pose;
  geometry_msgs::msg::PoseStamped left_target_pose;

  // 클래스 멤버(프라이빗) 추가
  std::string right_fjt_topic_;
  std::string left_fjt_topic_;

  std::string ur3_base_frame_;
  std::string rb3_base_frame_;

  // // ---- ROTBLOCK done notify server ----
  // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr rotblock_done_srv_;
  // std::atomic<uint64_t> rotblock_notify_count_{0};
  // rclcpp::Time last_rotblock_notify_time_;

  // // ---- MAINSEQ done notify server ----
  // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mainseq_done_srv_;
  // std::atomic<uint64_t> mainseq_notify_count_{0};
  // rclcpp::Time last_mainseq_notify_time_;

  // // ★ ADDED: 4/5 step done notify server
  // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr step4_done_srv_;
  // std::atomic<uint64_t> step4_notify_count_{0};
  // rclcpp::Time last_step4_notify_time_;

  // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr step5_done_srv_;
  // std::atomic<uint64_t> step5_notify_count_{0};
  // rclcpp::Time last_step5_notify_time_;

  void movetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::Movetcppos::Request> request,
      std::shared_ptr<dual_arm_msg::srv::Movetcppos::Response> response)
  {
      RCLCPP_INFO(this->get_logger(), "Planning tcp pose (dual_arm IK)...");

      move_group_arm_bimanual->setMaxVelocityScalingFactor(1.0);
      move_group_arm_bimanual->setMaxAccelerationScalingFactor(1.0);

      // Get bimanual planning frame
      std::string planning_frame = move_group_arm_bimanual->getPlanningFrame();
      RCLCPP_INFO(this->get_logger(), "Dual-arm planning frame: %s", planning_frame.c_str());

      // 준비: tf2 Buffer + Listener
      tf2_ros::Buffer tf_buffer(this->get_clock());
      tf2_ros::TransformListener tf_listener(tf_buffer);

      // Step 1: UR3e IK
      move_group_arm_ur3e->setStartStateToCurrentState();

      // UR 회전벡터 (Rx, Ry, Rz) → Quaternion 변환
      Eigen::Vector3d rotvec(request->left_rx, request->left_ry, request->left_rz);
      double angle = rotvec.norm();
      Eigen::Vector3d axis = (angle < 1e-6) ? Eigen::Vector3d(1, 0, 0) : rotvec.normalized();

      Eigen::AngleAxisd angle_axis(angle, axis);
      Eigen::Quaterniond quat(angle_axis);

      // tf2 Quaternion으로 변환
      tf2::Quaternion left_orientation(quat.x(), quat.y(), quat.z(), quat.w());

      // geometry_msgs 쿼터니언 메시지로 변환
      geometry_msgs::msg::Quaternion left_ros_orientation = tf2::toMsg(left_orientation);

      // PoseStamped 생성 및 세팅
      // geometry_msgs::msg::PoseStamped left_target_pose;
      left_target_pose.header.frame_id = ur3_base_frame_;
      left_target_pose.header.stamp = this->now();
      left_target_pose.pose.orientation = left_ros_orientation;
      left_target_pose.pose.position.x = request->left_x;
      left_target_pose.pose.position.y = request->left_y;
      left_target_pose.pose.position.z = request->left_z;

      geometry_msgs::msg::PoseStamped left_target_in_planning;
      try {
          left_target_in_planning = tf_buffer.transform(left_target_pose, planning_frame, tf2::durationFromSec(0.1));
      } catch (tf2::TransformException &ex) {
          RCLCPP_ERROR(this->get_logger(), "Transform error for UR3e: %s", ex.what());
          response->success = false;
          response->message = "UR3e transform failed.";
          return;
      }

      moveit::core::RobotState left_ik_state(*move_group_arm_ur3e->getCurrentState());
      std::vector<double> left_current_values;
      move_group_arm_ur3e->getCurrentState()->copyJointGroupPositions(joint_model_group_ur3e, left_current_values);
      left_ik_state.setJointGroupPositions(joint_model_group_ur3e, left_current_values);

      bool left_found_ik = left_ik_state.setFromIK(
          joint_model_group_ur3e,
          left_target_in_planning.pose,
          "tool0",
          0.1
      );

      if (!left_found_ik)
      {
          RCLCPP_ERROR(this->get_logger(), "UR3e IK solution not found.");
          response->success = false;
          response->message = "UR3e IK failed.";
          return;
      }

      std::vector<double> left_ik_joint_values;
      left_ik_state.copyJointGroupPositions(joint_model_group_ur3e, left_ik_joint_values);

      // Step 2: RB3 IK
      move_group_arm_rb3->setStartStateToCurrentState();

      tf2::Quaternion right_orientation;
      right_orientation.setRPY(request->right_rx, request->right_ry, request->right_rz);
      geometry_msgs::msg::Quaternion right_ros_orientation = tf2::toMsg(right_orientation);

      // geometry_msgs::msg::PoseStamped right_target_pose;
      right_target_pose.header.frame_id = rb3_base_frame_;
      right_target_pose.header.stamp = this->now();
      right_target_pose.pose.orientation = right_ros_orientation;
      right_target_pose.pose.position.x = request->right_x;
      right_target_pose.pose.position.y = request->right_y;
      right_target_pose.pose.position.z = request->right_z;

      geometry_msgs::msg::PoseStamped right_target_in_planning;
      try {
          right_target_in_planning = tf_buffer.transform(right_target_pose, planning_frame, tf2::durationFromSec(0.1));
      } catch (tf2::TransformException & ex) {
          RCLCPP_ERROR(this->get_logger(), "Transform error for RB3: %s", ex.what());
          response->success = false;
          response->message = "RB3 transform failed.";
          return;
      }

      moveit::core::RobotState right_ik_state(*move_group_arm_rb3->getCurrentState());
      std::vector<double> right_current_values;
      move_group_arm_rb3->getCurrentState()->copyJointGroupPositions(joint_model_group_rb3, right_current_values);
      right_ik_state.setJointGroupPositions(joint_model_group_rb3, right_current_values);

      bool right_found_ik = right_ik_state.setFromIK(
          joint_model_group_rb3,
          right_target_in_planning.pose,
          "tcp",
          0.1
      );

      if (!right_found_ik)
      {
          RCLCPP_ERROR(this->get_logger(), "RB3 IK solution not found.");
          response->success = false;
          response->message = "RB3 IK failed.";
          return;
      }

      std::vector<double> right_ik_joint_values;
      right_ik_state.copyJointGroupPositions(joint_model_group_rb3, right_ik_joint_values);

      // Step 3: dual_arm joint target 만들기
      std::map<std::string, double> dual_arm_joint_goal;

      const std::vector<std::string>& left_joint_names = joint_model_group_ur3e->getVariableNames();
      const std::vector<std::string>& right_joint_names = joint_model_group_rb3->getVariableNames();

      for (size_t i = 0; i < left_joint_names.size(); ++i)
      {
          dual_arm_joint_goal[left_joint_names[i]] = left_ik_joint_values[i];
      }

      for (size_t i = 0; i < right_joint_names.size(); ++i)
      {
          dual_arm_joint_goal[right_joint_names[i]] = right_ik_joint_values[i];
      }

      // Step 4: dual_arm planning + execute
      move_group_arm_bimanual->setStartStateToCurrentState();
      move_group_arm_bimanual->setJointValueTarget(dual_arm_joint_goal);

      moveit::planning_interface::MoveGroupInterface::Plan bimanual_plan;
      bool success = (move_group_arm_bimanual->plan(bimanual_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success)
      {
          RCLCPP_INFO(this->get_logger(), "Dual-arm plan successful, executing...");
          move_group_arm_bimanual->execute(bimanual_plan);

          response->success = true;
          response->message = "Dual-arm moved to pregrasp pose successfully.";
      }
      else
      {
          RCLCPP_ERROR(this->get_logger(), "Dual-arm planning failed.");
          response->success = false;
          response->message = "Dual-arm planning failed.";
      }
  }

  // -------------------- 오른팔(RB3): plan + FJT execute --------------------
  void rb3movetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::MovetcpposRB3::Request> request,
                             std::shared_ptr<dual_arm_msg::srv::MovetcpposRB3::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "RB3 plan + FJT exec...");

    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    move_group_arm_rb3->setStartStateToCurrentState();

    tf2::Quaternion rq; rq.setRPY(request->right_rx, request->right_ry, request->right_rz);
    geometry_msgs::msg::Quaternion rqo = tf2::toMsg(rq);

    right_target_pose.header.frame_id = rb3_base_frame_;
    right_target_pose.header.stamp = this->now();
    right_target_pose.pose.orientation = rqo;
    right_target_pose.pose.position.x = request->right_x;
    right_target_pose.pose.position.y = request->right_y;
    right_target_pose.pose.position.z = request->right_z;

    geometry_msgs::msg::PoseStamped rp;
    try { rp = tf_buffer.transform(right_target_pose, move_group_arm_rb3->getPlanningFrame(), tf2::durationFromSec(0.1)); }
    catch (tf2::TransformException &ex) { response->success=false; response->message=std::string("RB3 transform failed: ")+ex.what(); return; }

    moveit::core::RobotState rs(*move_group_arm_rb3->getCurrentState());
    std::vector<double> rv; move_group_arm_rb3->getCurrentState()->copyJointGroupPositions(joint_model_group_rb3, rv);
    rs.setJointGroupPositions(joint_model_group_rb3, rv);
    if (!rs.setFromIK(joint_model_group_rb3, rp.pose, "tcp", 0.1)) { response->success=false; response->message="RB3 IK failed."; return; }

    std::vector<double> rj; rs.copyJointGroupPositions(joint_model_group_rb3, rj);
    std::map<std::string,double> rg; const auto &rn = joint_model_group_rb3->getVariableNames(); for(size_t i=0;i<rn.size();++i) rg[rn[i]] = rj[i];
    move_group_arm_rb3->setJointValueTarget(rg);

    move_group_arm_rb3->setMaxVelocityScalingFactor(1.0);
    move_group_arm_rb3->setMaxAccelerationScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan rplan;
    if (move_group_arm_rb3->plan(rplan) != moveit::core::MoveItErrorCode::SUCCESS) { response->success=false; response->message="Right-arm planning failed."; return; }

    auto start_time = this->now() + rclcpp::Duration::from_seconds(0.2);
    sendFJTAsync(right_fjt_client_, rplan.trajectory_.joint_trajectory, start_time, "RIGHT");

    response->success = true;
    response->message = "Right-arm execution started via FJT (non-blocking).";
  }

  // -------------------- 왼팔(UR3e): plan + FJT execute --------------------
  void ur3emovetcpposCallback(const std::shared_ptr<dual_arm_msg::srv::MovetcpposUR3::Request> request,
                              std::shared_ptr<dual_arm_msg::srv::MovetcpposUR3::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "UR3e plan + FJT exec...");

    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    move_group_arm_ur3e->setStartStateToCurrentState();

    Eigen::Vector3d rv(request->left_rx, request->left_ry, request->left_rz);
    double angle = rv.norm();
    Eigen::Vector3d axis = (angle < 1e-6) ? Eigen::Vector3d(1, 0, 0) : rv.normalized();
    Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));
    geometry_msgs::msg::Quaternion qmsg = tf2::toMsg(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));

    left_target_pose.header.frame_id = ur3_base_frame_;
    left_target_pose.header.stamp = this->now();
    left_target_pose.pose.orientation = qmsg;
    left_target_pose.pose.position.x = request->left_x;
    left_target_pose.pose.position.y = request->left_y;
    left_target_pose.pose.position.z = request->left_z;

    geometry_msgs::msg::PoseStamped lp;
    try { lp = tf_buffer.transform(left_target_pose, move_group_arm_ur3e->getPlanningFrame(), tf2::durationFromSec(0.1)); }
    catch (tf2::TransformException &ex) { response->success=false; response->message=std::string("UR3e transform failed: ")+ex.what(); return; }

    moveit::core::RobotState ls(*move_group_arm_ur3e->getCurrentState());
    std::vector<double> lv; move_group_arm_ur3e->getCurrentState()->copyJointGroupPositions(joint_model_group_ur3e, lv);
    ls.setJointGroupPositions(joint_model_group_ur3e, lv);
    if (!ls.setFromIK(joint_model_group_ur3e, lp.pose, "tool0", 0.1)) { response->success=false; response->message="UR3e IK failed."; return; }

    std::vector<double> lj; ls.copyJointGroupPositions(joint_model_group_ur3e, lj);
    std::map<std::string,double> lg; const auto &ln = joint_model_group_ur3e->getVariableNames(); for(size_t i=0;i<ln.size();++i) lg[ln[i]] = lj[i];
    move_group_arm_ur3e->setJointValueTarget(lg);

    move_group_arm_ur3e->setMaxVelocityScalingFactor(1.0);
    move_group_arm_ur3e->setMaxAccelerationScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan lplan;
    if (move_group_arm_ur3e->plan(lplan) != moveit::core::MoveItErrorCode::SUCCESS) { response->success=false; response->message="Left-arm planning failed."; return; }

    auto start_time = this->now() + rclcpp::Duration::from_seconds(0.2);
    sendFJTAsync(left_fjt_client_, lplan.trajectory_.joint_trajectory, start_time, "LEFT");

    std::stringstream ss;
    ss << "Left execution started via FJT (non-blocking). Pos(x,y,z): "
       << request->left_x << ", " << request->left_y << ", " << request->left_z;
    response->success = true;
    response->message = ss.str();
  }

  // -------------------- Gripper --------------------
  // bimanual_service.cpp - 바뀐 그리퍼 서비스(즉시 ACK)
  void gripopenCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // ❶ 바로 응답(ACK) 반환
    response->success = true;
    response->message = "Gripper open dispatched (fire-and-forget).";

    // ❷ 실제 하드웨어 명령은 백그라운드에서
    std::thread([node = gripper_node_, cli = gripper_client_]() {
      (void)gripper_client::send_gripper_command(node, cli, 0);
    }).detach();
  }

  void gripcloseCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = true;
    response->message = "Gripper close dispatched (fire-and-forget).";

    std::thread([node = gripper_node_, cli = gripper_client_]() {
      (void)gripper_client::send_gripper_command(node, cli, 255);
    }).detach();
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
