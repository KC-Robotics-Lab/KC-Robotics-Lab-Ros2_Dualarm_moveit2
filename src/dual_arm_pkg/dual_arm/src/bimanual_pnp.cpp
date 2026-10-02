// #include <memory>
// #include <thread>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit/kinematics_base/kinematics_base.h>
// #include <Eigen/Geometry>
// #include <array>

// #include "gripper_client.hpp"

// #include "ur_pick_and_place_msgs/srv/movetcppos.hpp"
// #include "rbpodo_msgs/srv/movetcppos.hpp"
// #include "dual_arm_msg/srv/movetcppos.hpp"
// #include "dual_arm_msg/srv/movetcppos_rb3.hpp"
// #include "dual_arm_msg/srv/movetcppos_ur3.hpp"

// #include "inspire_hand_interface/action/set_angle.hpp"

// using SetAngle = inspire_hand_interface::action::SetAngle;
// using GoalHandleSetAngle = rclcpp_action::ClientGoalHandle<SetAngle>;

// uint8_t hand_id = 1;
// std::array<uint16_t, 6> hand_angles;

// class SequentialServiceCaller : public rclcpp::Node
// {
//     public:
//         SequentialServiceCaller() : Node("sequential_service_caller")
//         {
//             using namespace std::chrono_literals;

//             // 클라이언트 생성
//             client_bimanual_tcppos = this->create_client<dual_arm_msg::srv::Movetcppos>("zmk_move_to_tcppos");
//             client_bimanual_home = this->create_client<std_srvs::srv::Trigger>("zmk_move_to_home");
//             client_bimanual_rb3_tcppos = this->create_client<dual_arm_msg::srv::MovetcpposRB3>("zmk_tcppos_rb3");
//             client_bimanual_ur3e_tcppos = this->create_client<dual_arm_msg::srv::MovetcpposUR3>("zmk_tcppos_ur3e");
//             client_gripper_close = this->create_client<std_srvs::srv::Trigger>("zmk_girp_close");
//             client_gripper_open = this->create_client<std_srvs::srv::Trigger>("zmk_girp_open");
//             client_hand_angle = rclcpp_action::create_client<SetAngle>(this, "set_angle");

//             move_seq();
//         }

//     private:
//         rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedPtr client_bimanual_tcppos;
//         rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_bimanual_home;
//         rclcpp::Client<dual_arm_msg::srv::MovetcpposRB3>::SharedPtr client_bimanual_rb3_tcppos;
//         rclcpp::Client<dual_arm_msg::srv::MovetcpposUR3>::SharedPtr client_bimanual_ur3e_tcppos;
//         rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_gripper_close;
//         rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_gripper_open;

//         rclcpp_action::Client<SetAngle>::SharedPtr client_hand_angle;

//         void move_seq()
//         {
//             // 순차 실행 시작
//             wait_for_service(client_gripper_open, "zmk_girp_open");
//             call_zmk_gripper_open();
//             RCLCPP_INFO(this->get_logger(), "-------------------gripper open------------------");
//             // hand_angles = {1000, 1000, 1000, 1000, 1000, 0};
//             // Hand_send_goal();
//             // RCLCPP_INFO(this->get_logger(), "-------------------hand open------------------");
//             wait_for_service(client_bimanual_home, "zmk_move_to_home");
//             call_zmk_move_to_home();    //home
//             RCLCPP_INFO(this->get_logger(), "-------------------move to home------------------");
//             // wait_for_service(client_bimanual_tcppos, "zmk_move_to_tcppos");
//             // call_zmk_move_to_bimanual_tcppos(0.01, -0.41, 0.264, 0.0, -3.14, 0.0, -0.026, -0.417, 0.21, 2.044, 0.112, 1.412); 
//             // RCLCPP_INFO(this->get_logger(), "-------------------move to pick and place position------------------");
//             wait_for_service(client_bimanual_rb3_tcppos, "zmk_tcppos_rb3");
//             call_zmk_tcppos_rb3(-0.026, -0.417, 0.175, 2.044, 0.112, 1.412);    //RB3 골프공 집는 위치
//             RCLCPP_INFO(this->get_logger(), "-------------------right arm down------------------");
//             // hand_angles = {1000, 1000, 480, 420, 850, 0};
//             // Hand_send_goal();                                                   //little, ring, middle, index, thumb, thumb rotation
//             // RCLCPP_INFO(this->get_logger(), "-------------------hand close------------------");
//             wait_for_service(client_bimanual_rb3_tcppos, "zmk_tcppos_rb3");
//             call_zmk_tcppos_rb3(-0.026, -0.417, 0.21, 2.044, 0.112, 1.412);     //RB3 골프공 잡는 위치 조금 위
//             RCLCPP_INFO(this->get_logger(), "-------------------right arm up------------------");
//             // wait_for_service(client_bimanual_tcppos, "zmk_move_to_tcppos");
//             // call_zmk_move_to_bimanual_tcppos(-0.21996, -0.28729, 0.42326, 2.127, 2.284, 0.06, 0.2325, -0.26241, 0.12236, -1.524, -0.0544, -1.542);  // RB3 / UR3 골프공 전달 위치
//             // RCLCPP_INFO(this->get_logger(), "-------------------move to give and take position------------------");
//             wait_for_service(client_bimanual_ur3e_tcppos, "zmk_tcppos_ur3e");
//             call_zmk_tcppos_ur3e(-0.21996, -0.28729, 0.36662, 2.127, 2.284, 0.06);         // UR3 골프공 잡는 위치
//             RCLCPP_INFO(this->get_logger(), "-------------------left arm down------------------");
//             wait_for_service(client_gripper_close, "zmk_girp_close");
//             call_zmk_gripper_close();       // 골프공 잡기
//             RCLCPP_INFO(this->get_logger(), "-------------------gripper close------------------");
//             // hand_angles = {1000, 1000, 1000, 1000, 1000, 0};
//             // Hand_send_goal();
//             // RCLCPP_INFO(this->get_logger(), "-------------------hand open------------------");
//             // wait_for_service(client_bimanual_tcppos, "zmk_move_to_tcppos");
//             // call_zmk_move_to_bimanual_tcppos(-0.21996, -0.28729, 0.42326, 2.127, 2.284, 0.06, 0.1825, -0.26241, 0.12236, -1.524, -0.0544, -1.542);  //RB3는 뒤로 / UR3는 위로
//             // RCLCPP_INFO(this->get_logger(), "-------------------left arm up / right arm back------------------");
//             // wait_for_service(client_bimanual_tcppos, "zmk_move_to_tcppos");
//             // call_zmk_move_to_bimanual_tcppos(0.01, -0.41, 0.264, 0.0, -3.14, 0.0, -0.026, -0.417, 0.21, 2.044, 0.112, 1.412);       // RB3는 골프공 pick한 위치 / UR3는 Place할 위치
//             // RCLCPP_INFO(this->get_logger(), "-------------------move to pick and place position------------------");
//             wait_for_service(client_bimanual_ur3e_tcppos, "zmk_tcppos_ur3e");
//             call_zmk_tcppos_ur3e(0.01, -0.41, 0.180, 0.0, -3.14, 0.0);
//             RCLCPP_INFO(this->get_logger(), "-------------------left arm down------------------");
//             wait_for_service(client_gripper_open, "zmk_girp_open");
//             call_zmk_gripper_open();
//             RCLCPP_INFO(this->get_logger(), "-------------------gripper open------------------");
//             wait_for_service(client_bimanual_ur3e_tcppos, "zmk_tcppos_ur3e");
//             call_zmk_tcppos_ur3e(0.01, -0.41, 0.264, 0.0, -3.14, 0.0);      // UR 조금 위로
//             RCLCPP_INFO(this->get_logger(), "-------------------left arm up------------------");
//             wait_for_service(client_bimanual_home, "zmk_move_to_home");
//             call_zmk_move_to_home();    //home
//             RCLCPP_INFO(this->get_logger(), "-------------------move to home------------------");
//             RCLCPP_INFO(this->get_logger(), "-------------------work done------------------");
//         }

//         void wait_for_service(const rclcpp::ClientBase::SharedPtr &client, const std::string &name)
//         {
//             if (!client->wait_for_service(std::chrono::seconds(2)))
//             {
//             RCLCPP_ERROR(this->get_logger(), "Service [%s] is not available.", name.c_str());
//             rclcpp::shutdown();
//             }
//         }

//         void call_zmk_move_to_bimanual_tcppos(double l_x, double l_y, double l_z, double l_rx, double l_ry, double l_rz,
//             double r_x, double r_y, double r_z, double r_rx, double r_ry, double r_rz)
//         {
//             auto req = std::make_shared<dual_arm_msg::srv::Movetcppos::Request>();
//             // 예시 위치 설정 (원하는 pose로 변경 가능)
//             req->left_x = l_x; req->left_y = l_y; req->left_z = l_z;
//             req->left_rx = l_rx; req->left_ry = l_ry; req->left_rz = l_rz;
//             req->right_x = r_x; req->right_y = r_y; req->right_z = r_z;
//             req->right_rx = r_rx; req->right_ry = r_ry; req->right_rz = r_rz;

//             auto future = client_bimanual_tcppos->async_send_request(req);

//             if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
//                 rclcpp::FutureReturnCode::SUCCESS)
//             {
//                 if (future.valid())
//                 {
//                     auto response = future.get();  // .get()은 반드시 한 번만 호출!
//                     if (response->success)
//                     {
//                         RCLCPP_INFO(this->get_logger(), "zmk_move_to_tcppos success");
//                     }
//                     else
//                     {
//                         RCLCPP_ERROR(this->get_logger(), "zmk_move_to_tcppos failed: %s", response->message.c_str());
//                     }
//                 }
//                 else
//                 {
//                     RCLCPP_ERROR(this->get_logger(), "zmk_move_to_tcppos future is not valid.");
//                 }
//             }
//             else
//             {
//                 RCLCPP_ERROR(this->get_logger(), "zmk_move_to_tcppos service timeout");
//             }
//             rclcpp::sleep_for(std::chrono::seconds(1));
//         }

//         void call_zmk_move_to_home()
//         {
//             auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
//             auto future = client_bimanual_home->async_send_request(req);
        
//             if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
//                 rclcpp::FutureReturnCode::SUCCESS)
//             {
//                 if (future.valid())
//                 {
//                     auto response = future.get();
//                     if (response->success)
//                     {
//                         RCLCPP_INFO(this->get_logger(), "zmk_move_to_home success");
//                     }
//                     else
//                     {
//                         RCLCPP_ERROR(this->get_logger(), "zmk_move_to_home failed: %s", response->message.c_str());
//                     }
//                 }
//                 else
//                 {
//                     RCLCPP_ERROR(this->get_logger(), "zmk_move_to_home future is not valid.");
//                 }
//             }
//             else
//             {
//                 RCLCPP_ERROR(this->get_logger(), "zmk_move_to_home service timeout");
//             }
//             rclcpp::sleep_for(std::chrono::seconds(1));
//         }
        

//         void call_zmk_tcppos_rb3(double x, double y, double z, double rx, double ry, double rz)
//         {
//             auto req = std::make_shared<dual_arm_msg::srv::MovetcpposRB3::Request>();
//             req->right_x = x; req->right_y = y; req->right_z = z;
//             req->right_rx = rx; req->right_ry = ry; req->right_rz = rz;

//             auto future = client_bimanual_rb3_tcppos->async_send_request(req);

//             if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
//                 rclcpp::FutureReturnCode::SUCCESS)
//             {
//                 if (future.valid())
//                 {
//                     auto response = future.get();  // .get()은 반드시 한 번만 호출!
//                     if (response->success)
//                     {
//                         RCLCPP_INFO(this->get_logger(), "zmk_tcppos_rb3 success");
//                     }
//                     else
//                     {
//                         RCLCPP_ERROR(this->get_logger(), "zmk_tcppos_rb3 failed: %s", response->message.c_str());
//                     }
//                 }
//                 else
//                 {
//                     RCLCPP_ERROR(this->get_logger(), "zmk_tcppos_rb3 future is not valid.");
//                 }
//             }
//             else
//             {
//                 RCLCPP_ERROR(this->get_logger(), "zmk_tcppos_rb3 service timeout");
//             }
//             rclcpp::sleep_for(std::chrono::seconds(1));
//         }

//         void call_zmk_tcppos_ur3e(double x, double y, double z, double rx, double ry, double rz)
//         {
//             auto req = std::make_shared<dual_arm_msg::srv::MovetcpposUR3::Request>();
//             req->left_x = x; req->left_y = y; req->left_z = z;
//             req->left_rx = rx; req->left_ry = ry; req->left_rz = rz;

//             auto future = client_bimanual_ur3e_tcppos->async_send_request(req);

//             if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
//                 rclcpp::FutureReturnCode::SUCCESS)
//             {
//                 if (future.valid())
//                 {
//                     auto response = future.get();
//                     if (response->success)
//                     {
//                         RCLCPP_INFO(this->get_logger(), "zmk_tcppos_ur3e success");
//                     }
//                     else
//                     {
//                         RCLCPP_ERROR(this->get_logger(), "zmk_tcppos_ur3e failed: %s", response->message.c_str());
//                     }
//                 }
//                 else
//                 {
//                     RCLCPP_ERROR(this->get_logger(), "zmk_tcppos_ur3e future is not valid.");
//                 }
//             }
//             else
//             {
//                 RCLCPP_ERROR(this->get_logger(), "zmk_tcppos_ur3e service timeout");
//             }
//             rclcpp::sleep_for(std::chrono::seconds(1));
//         }

//         void call_zmk_gripper_close()
//         {
//             auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
//             auto future = client_gripper_close->async_send_request(req);

//             if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
//                 rclcpp::FutureReturnCode::SUCCESS)
//             {
//                 if (future.valid())
//                 {
//                     auto response = future.get();
//                     if (response->success)
//                     {
//                         RCLCPP_INFO(this->get_logger(), "zmk_gripper_close success");
//                     }
//                     else
//                     {
//                         RCLCPP_ERROR(this->get_logger(), "zmk_gripper_close failed: %s", response->message.c_str());
//                     }
//                 }
//                 else
//                 {
//                     RCLCPP_ERROR(this->get_logger(), "zmk_gripper_close future is not valid.");
//                 }
//             }
//             else
//             {
//                 RCLCPP_ERROR(this->get_logger(), "zmk_gripper_close service timeout");
//             }
//             rclcpp::sleep_for(std::chrono::seconds(1));
//         }

//         void call_zmk_gripper_open()
//         {
//             auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
//             auto future = client_gripper_open->async_send_request(req);

//             if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
//                 rclcpp::FutureReturnCode::SUCCESS)
//             {
//                 if (future.valid())
//                 {
//                     auto response = future.get();
//                     if (response->success)
//                     {
//                         RCLCPP_INFO(this->get_logger(), "zmk_gripper_open success");
//                     }
//                     else
//                     {
//                         RCLCPP_ERROR(this->get_logger(), "zmk_gripper_open failed: %s", response->message.c_str());
//                     }
//                 }
//                 else
//                 {
//                     RCLCPP_ERROR(this->get_logger(), "zmk_gripper_open future is not valid.");
//                 }
//             }
//             else
//             {
//                 RCLCPP_ERROR(this->get_logger(), "zmk_gripper_open service timeout");
//             }
//             rclcpp::sleep_for(std::chrono::seconds(1));
//         }

//         void Hand_send_goal()
//         {
//             // timer_->cancel();  // 한 번만 실행
        
//             if (!client_hand_angle->wait_for_action_server(std::chrono::seconds(2))) {
//                 RCLCPP_ERROR(this->get_logger(), "SetAngle action server not available.");
//                 return;
//             }
        
//             auto goal_msg = SetAngle::Goal();
//             goal_msg.hand_id = hand_id;
//             goal_msg.angle0 = hand_angles[0];
//             goal_msg.angle1 = hand_angles[1];
//             goal_msg.angle2 = hand_angles[2];
//             goal_msg.angle3 = hand_angles[3];
//             goal_msg.angle4 = hand_angles[4];
//             goal_msg.angle5 = hand_angles[5];
        
//             RCLCPP_INFO(this->get_logger(), "Sending SetAngle goal...");
        
//             auto goal_future = client_hand_angle->async_send_goal(goal_msg);
        
//             // ⚠️ 기존 spin_until_future_complete(this->...) 대신 임시 executor 사용
//             rclcpp::executors::SingleThreadedExecutor exec;
//             exec.add_node(this->get_node_base_interface());
        
//             if (exec.spin_until_future_complete(goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
//                 RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
//                 return;
//             }
        
//             auto goal_handle = goal_future.get();
//             if (!goal_handle) {
//                 RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//                 return;
//             }
        
//             RCLCPP_INFO(this->get_logger(), "Goal accepted, waiting for result...");
        
//             auto result_future = client_hand_angle->async_get_result(goal_handle);
        
//             if (exec.spin_until_future_complete(result_future) != rclcpp::FutureReturnCode::SUCCESS) {
//                 RCLCPP_ERROR(this->get_logger(), "Get result call failed");
//                 return;
//             }
        
//             auto result = result_future.get();
        
//             switch (result.code) {
//                 case rclcpp_action::ResultCode::SUCCEEDED:
//                     RCLCPP_INFO(this->get_logger(), "SetAngle succeeded: %d", result.result->success);
//                     break;
//                 case rclcpp_action::ResultCode::ABORTED:
//                     RCLCPP_ERROR(this->get_logger(), "SetAngle goal was aborted");
//                     break;
//                 case rclcpp_action::ResultCode::CANCELED:
//                     RCLCPP_ERROR(this->get_logger(), "SetAngle goal was canceled");
//                     break;
//                 default:
//                     RCLCPP_ERROR(this->get_logger(), "Unknown result code from SetAngle");
//                     break;
//             }
//             rclcpp::sleep_for(std::chrono::seconds(1));
//         }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<SequentialServiceCaller>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


//--------------------------------핸드 추가 전 코드---------------------------------

// #include <memory>
// #include <vector>
// #include <string>
// #include <chrono>
// #include <unordered_map>
// #include <functional>
// #include <optional>
// #include <atomic>
// #include <sstream>
// #include <iomanip>

// #include <rclcpp/rclcpp.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <action_msgs/msg/goal_status_array.hpp>

// #include "dual_arm_msg/srv/movetcppos_rb3.hpp"
// #include "dual_arm_msg/srv/movetcppos_ur3.hpp"

// using namespace std::chrono_literals;
// using action_msgs::msg::GoalStatusArray;
// using action_msgs::msg::GoalStatus;

// using RB3Srv = dual_arm_msg::srv::MovetcpposRB3;
// using UR3Srv = dual_arm_msg::srv::MovetcpposUR3;

// struct Pose6 { double x, y, z, rx, ry, rz; };

// enum class GripperAction { NONE = 0, OPEN, CLOSE };

// class MoveSequenceClient : public rclcpp::Node
// {
// public:
//   MoveSequenceClient()
//   : Node("move_sequence_client_event_driven")
//   , t0_steady_(std::chrono::steady_clock::now())
//   {
//     right_fjt_base_ = declare_parameter<std::string>(
//       "right_fjt_base", "/joint_trajectory_controller/follow_joint_trajectory");
//     left_fjt_base_ = declare_parameter<std::string>(
//       "left_fjt_base", "/scaled_joint_trajectory_controller/follow_joint_trajectory");

//     cli_rb3_ = this->create_client<RB3Srv>("zmk_tcppos_rb3");
//     cli_ur3_ = this->create_client<UR3Srv>("zmk_tcppos_ur3e");

//     cli_grip_open_  = this->create_client<std_srvs::srv::Trigger>("zmk_girp_open");
//     cli_grip_close_ = this->create_client<std_srvs::srv::Trigger>("zmk_girp_close");

//     // 시퀀스
//     seq_rb3_ = {
//       {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708}, // (0)
//       {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (1)
//       {-0.0407,  -0.44568, 0.14802, 1.7228145,   0.17715092,  1.80205245}, // (2)
//       { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (3)
//       { 0.11564, -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (4)
//       { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (5)
//       {-0.0407,  -0.44568, 0.15102, 1.7228145,   0.17715092,  1.80205245}, // (6)
//       {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (7)
//     };
//     seq_ur3_ = {
//       { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)
//       { -0.140,   -0.410,   0.165,   0.0,   -3.14,  0.0 },      // (1)
//       { -0.13056, -0.32030, 0.46642, 2.249,  2.489, -2.523 },   // (2)
//       { -0.18056, -0.32030, 0.46642, 2.249,  2.489, -2.523 },   // (3)
//       { -0.13056, -0.32030, 0.46642, 2.249,  2.489, -2.523 },   // (4)
//       { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (5)
//     };

//     // 그리퍼 훅 : 그리퍼 시퀀스 추가하려면 여기에 추가, 버퍼 숫자 : 시퀀스 번호
//     ur3_grip_before_[0] = GripperAction::OPEN;
//     ur3_grip_after_[1]  = GripperAction::CLOSE;
//     ur3_grip_after_[3]  = GripperAction::OPEN;

//     // 배리어
//     sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/3});
//     sync_points_.push_back(PairSync{/*rb3_idx=*/4, /*ur3_idx=*/3, /*rb3_next=*/5, /*ur3_next=*/4});

//     start_timer_ = this->create_wall_timer(300ms, [this] {
//       if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s) ||
//           !cli_grip_open_->wait_for_service(0s) || !cli_grip_close_->wait_for_service(0s)) {
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
//           "[%s] Waiting for services (rb3/ur3e/gripper)...", ts().c_str());
//         return;
//       }
//       const auto right_status = resolve_status_topic(right_fjt_base_);
//       const auto left_status  = resolve_status_topic(left_fjt_base_);
//       if (right_status.empty() || left_status.empty()) {
//         RCLCPP_ERROR(get_logger(),
//           "[%s] Could not resolve action status topics. Check controller namespaces. "
//           "right_base=%s left_base=%s", ts().c_str(), right_fjt_base_.c_str(), left_fjt_base_.c_str());
//         return;
//       }
//       RCLCPP_INFO(get_logger(), "[%s] [RESOLVED] RIGHT status: %s", ts().c_str(), right_status.c_str());
//       RCLCPP_INFO(get_logger(), "[%s] [RESOLVED] LEFT  status: %s", ts().c_str(), left_status.c_str());

//       sub_right_status_ = this->create_subscription<GoalStatusArray>(
//         right_status, rclcpp::QoS(50),
//         std::bind(&MoveSequenceClient::on_right_status, this, std::placeholders::_1));
//       sub_left_status_ = this->create_subscription<GoalStatusArray>(
//         left_status, rclcpp::QoS(50),
//         std::bind(&MoveSequenceClient::on_left_status, this, std::placeholders::_1));

//       start_timer_->cancel();
//       start_sequence();
//     });
//   }

// private:
//   struct PairSync {
//     size_t rb3_idx;
//     size_t ur3_idx;
//     size_t rb3_next;
//     size_t ur3_next;
//     bool reached_rb3 = false;
//     bool reached_ur3 = false;
//     bool fired = false;
//   };
//   std::vector<PairSync> sync_points_;

//   std::optional<std::pair<size_t,size_t>> sync_inflight_;
//   bool rb3_inflight_done_ = false;
//   bool ur3_inflight_done_ = false;

//   rclcpp::Client<RB3Srv>::SharedPtr cli_rb3_;
//   rclcpp::Client<UR3Srv>::SharedPtr cli_ur3_;
//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_open_;
//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_close_;
//   rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_right_status_;
//   rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_left_status_;
//   rclcpp::TimerBase::SharedPtr start_timer_;
//   std::string right_fjt_base_, left_fjt_base_;
//   std::vector<Pose6> seq_rb3_, seq_ur3_;
//   size_t idx_rb3_ = 0, idx_ur3_ = 0;
//   bool rb3_waiting_ = false, ur3_waiting_ = false;
//   bool rb3_seen_active_ = false, ur3_seen_active_ = false;
//   bool rb3_done_all_ = false,  ur3_done_all_ = false;

//   std::unordered_map<size_t, GripperAction> ur3_grip_before_;
//   std::unordered_map<size_t, GripperAction> ur3_grip_after_;
//   std::unordered_map<size_t, GripperAction> rb3_grip_before_;
//   std::unordered_map<size_t, GripperAction> rb3_grip_after_;

//   rclcpp::TimerBase::SharedPtr sync_fire_timer_;

//   std::chrono::steady_clock::time_point t0_steady_;
//   std::chrono::steady_clock::time_point t_send_rb3_, t_send_ur3_;
//   std::chrono::steady_clock::time_point t_active_rb3_, t_active_ur3_;

//   std::string ts() const {
//     using namespace std::chrono;
//     auto now = steady_clock::now();
//     auto ms  = duration_cast<milliseconds>(now - t0_steady_).count();
//     std::ostringstream oss; oss << std::setw(9) << ms << "ms";
//     return oss.str();
//   }
//   static bool ends_with(const std::string &s, const std::string &suffix) {
//     return s.size() >= suffix.size() &&
//            s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
//   }
//   static bool any_active(const GoalStatusArray &arr) {
//     for (const auto &st : arr.status_list) {
//       if (st.status == GoalStatus::STATUS_ACCEPTED ||
//           st.status == GoalStatus::STATUS_EXECUTING ||
//           st.status == GoalStatus::STATUS_CANCELING) return true;
//     }
//     return false;
//   }

//   std::string resolve_status_topic(const std::string &base)
//   {
//     const auto a = base + "/_action/status";
//     const auto b = base + "/status";
//     auto graph = this->get_topic_names_and_types();
//     if (graph.find(a) != graph.end()) return a;
//     if (graph.find(b) != graph.end()) return b;
//     for (const auto &kv : graph) {
//       const auto &topic = kv.first;
//       if ((ends_with(topic, "/_action/status") || ends_with(topic, "/status")) &&
//           topic.find(base) != std::string::npos) {
//         return topic;
//       }
//     }
//     return std::string();
//   }

//   void start_sequence() {
//     RCLCPP_INFO(this->get_logger(), "[%s] === Sequence PREP ===", ts().c_str());
//     prepare_and_fire_first_pair();
//   }

//   void prepare_and_fire_first_pair() {
//     const bool need_rb3 = rb3_grip_before_.count(0) && rb3_grip_before_[0] != GripperAction::NONE;
//     const bool need_ur3 = ur3_grip_before_.count(0) && ur3_grip_before_[0] != GripperAction::NONE;

//     if (!need_rb3 && !need_ur3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] No BEFORE hooks at idx0 → FIRE BOTH step0 now", ts().c_str());
//       sync_inflight_ = std::make_pair(0u, 0u);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;
//       fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
//       return;
//     }

//     RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks at idx0: RB3=%s, UR3=%s",
//       ts().c_str(), need_rb3 ? "YES" : "NO", need_ur3 ? "YES" : "NO");

//     auto pending = std::make_shared<std::atomic<int>>(0);
//     if (need_rb3) pending->fetch_add(1);
//     if (need_ur3) pending->fetch_add(1);

//     auto after = [this, pending]() {
//       if (pending->fetch_sub(1) == 1) {
//         RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks done → FIRE BOTH step0", ts().c_str());
//         sync_inflight_ = std::make_pair(0u, 0u);
//         rb3_inflight_done_ = ur3_inflight_done_ = false;
//         fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
//       }
//     };

//     if (need_rb3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [PREP] RB3 BEFORE hook (idx0)", ts().c_str());
//       request_gripper(rb3_grip_before_[0], after);
//     }
//     if (need_ur3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [PREP] UR3e BEFORE hook (idx0)", ts().c_str());
//       request_gripper(ur3_grip_before_[0], after);
//     }
//   }

//   void fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks)
//   {
//     if (!apply_before_hooks) {
//       sync_fire_timer_ = this->create_wall_timer(
//         0ms,
//         [this, rb3_idx, ur3_idx](){
//           RCLCPP_INFO(this->get_logger(),
//             "[%s] FIRE BOTH NOW (RB3=%zu, UR3e=%zu) — same tick",
//             ts().c_str(), rb3_idx, ur3_idx);
//           send_rb3_step(rb3_idx);
//           send_ur3_step(ur3_idx);
//           sync_fire_timer_->cancel();
//         }
//       );
//       return;
//     }

//     size_t pending = 0;
//     bool do_rb3_before = rb3_grip_before_.count(rb3_idx) &&
//                         rb3_grip_before_[rb3_idx] != GripperAction::NONE;
//     bool do_ur3_before = ur3_grip_before_.count(ur3_idx) &&
//                         ur3_grip_before_[ur3_idx] != GripperAction::NONE;

//     auto after_one_done = [&]() {
//       if (--pending == 0) {
//         sync_fire_timer_ = this->create_wall_timer(
//           0ms,
//           [this, rb3_idx, ur3_idx](){
//             RCLCPP_INFO(this->get_logger(),
//               "[%s] FIRE BOTH (after hooks) (RB3=%zu, UR3e=%zu)", ts().c_str(), rb3_idx, ur3_idx);
//             send_rb3_step(rb3_idx);
//             send_ur3_step(ur3_idx);
//             sync_fire_timer_->cancel();
//           }
//         );
//       }
//     };

//     if (do_rb3_before) pending++;
//     if (do_ur3_before) pending++;
//     if (pending == 0) {
//       sync_fire_timer_ = this->create_wall_timer(
//         0ms, [this, rb3_idx, ur3_idx](){
//           RCLCPP_INFO(this->get_logger(),
//             "[%s] FIRE BOTH (no hooks) (RB3=%zu, UR3e=%zu)", ts().c_str(), rb3_idx, ur3_idx);
//           send_rb3_step(rb3_idx);
//           send_ur3_step(ur3_idx);
//           sync_fire_timer_->cancel();
//         }
//       );
//       return;
//     }
//     if (do_rb3_before) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] RB3 BEFORE hook (idx%zu)", ts().c_str(), rb3_idx);
//       request_gripper(rb3_grip_before_[rb3_idx], [after_one_done]{});
//     }
//     if (do_ur3_before) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] UR3e BEFORE hook (idx%zu)", ts().c_str(), ur3_idx);
//       request_gripper(ur3_grip_before_[ur3_idx], [after_one_done]{});
//     }
//   }

//   // ===== RB3 =====
//   void start_rb3_step(size_t i)
//   {
//     if (i >= seq_rb3_.size()) { if (!rb3_done_all_) { rb3_done_all_ = true; maybe_finish(); } return; }
//     if (rb3_grip_before_.count(i) && rb3_grip_before_[i] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB3] BEFORE hook at idx=%zu", ts().c_str(), i);
//       request_gripper(rb3_grip_before_[i], [this, i]{ send_rb3_step(i); });
//     } else {
//       send_rb3_step(i);
//     }
//   }
//   void send_rb3_step(size_t i)
//   {
//     if (i >= seq_rb3_.size()) return;
//     const auto &p = seq_rb3_[i];
//     auto req = std::make_shared<RB3Srv::Request>();
//     req->right_x = p.x; req->right_y = p.y; req->right_z = p.z;
//     req->right_rx = p.rx; req->right_ry = p.ry; req->right_rz = p.rz;

//     rb3_waiting_ = true;
//     rb3_seen_active_ = false;
//     t_send_rb3_ = std::chrono::steady_clock::now();

//     RCLCPP_INFO(this->get_logger(), "[%s] [RB3] SEND step %zu", ts().c_str(), i);
//     cli_rb3_->async_send_request(
//       req,
//       [this, i](rclcpp::Client<RB3Srv>::SharedFuture future) {
//         auto resp = future.get();
//         if (resp && resp->success) {
//           RCLCPP_INFO(this->get_logger(), "[%s] [RB3] step %zu SENT OK: %s",
//                       ts().c_str(), i, resp->message.c_str());
//         } else {
//           RCLCPP_ERROR(this->get_logger(), "[%s] [RB3] step %zu SEND FAILED", ts().c_str(), i);
//         }
//       });
//   }
//   void advance_rb3_after_done(size_t just_finished_idx)
//   {
//     if (rb3_grip_after_.count(just_finished_idx) &&
//         rb3_grip_after_[just_finished_idx] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB3] AFTER hook at idx=%zu", ts().c_str(), just_finished_idx);
//       request_gripper(rb3_grip_after_[just_finished_idx], [this]{ ++idx_rb3_; start_rb3_step(idx_rb3_); });
//     } else {
//       ++idx_rb3_;
//       start_rb3_step(idx_rb3_);
//     }
//   }

//   // ===== UR3e =====
//   void start_ur3_step(size_t i)
//   {
//     if (i >= seq_ur3_.size()) { if (!ur3_done_all_) { ur3_done_all_ = true; maybe_finish(); } return; }
//     if (ur3_grip_before_.count(i) && ur3_grip_before_[i] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] BEFORE hook at idx=%zu", ts().c_str(), i);
//       request_gripper(ur3_grip_before_[i], [this, i]{ send_ur3_step(i); });
//     } else {
//       send_ur3_step(i);
//     }
//   }
//   void send_ur3_step(size_t i)
//   {
//     if (i >= seq_ur3_.size()) return;
//     const auto &p = seq_ur3_[i];
//     auto req = std::make_shared<UR3Srv::Request>();
//     req->left_x = p.x; req->left_y = p.y; req->left_z = p.z;
//     req->left_rx = p.rx; req->left_ry = p.ry; req->left_rz = p.rz;

//     ur3_waiting_ = true;
//     ur3_seen_active_ = false;
//     t_send_ur3_ = std::chrono::steady_clock::now();

//     RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] SEND step %zu", ts().c_str(), i);
//     cli_ur3_->async_send_request(
//       req,
//       [this, i](rclcpp::Client<UR3Srv>::SharedFuture future) {
//         auto resp = future.get();
//         if (resp && resp->success) {
//           RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] step %zu SENT OK%s",
//                       ts().c_str(), i, resp->message.c_str());
//         } else {
//           RCLCPP_ERROR(this->get_logger(), "[%s] [UR3e] step %zu SEND FAILED", ts().c_str(), i);
//         }
//       });
//   }
//   void advance_ur3_after_done(size_t just_finished_idx)
//   {
//     if (ur3_grip_after_.count(just_finished_idx) &&
//         ur3_grip_after_[just_finished_idx] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] AFTER hook at idx=%zu", ts().c_str(), just_finished_idx);
//       request_gripper(ur3_grip_after_[just_finished_idx], [this]{ ++idx_ur3_; start_ur3_step(idx_ur3_); });
//     } else {
//       ++idx_ur3_;
//       start_ur3_step(idx_ur3_);
//     }
//   }

//   // ★FIX: inflight(배리어)로 보낸 스텝이 끝났을 때도 AFTER 훅을 수행
//   void run_after_hook_barrier_only(bool is_left, size_t finished_idx)
//   {
//     GripperAction act = GripperAction::NONE;
//     if (is_left) {
//       auto it = ur3_grip_after_.find(finished_idx);
//       if (it != ur3_grip_after_.end()) act = it->second;
//     } else {
//       auto it = rb3_grip_after_.find(finished_idx);
//       if (it != rb3_grip_after_.end()) act = it->second;
//     }
//     if (act != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(),
//         "[%s] [BARRIER AFTER] %s idx=%zu → %s",
//         ts().c_str(), is_left ? "UR3e" : "RB3", finished_idx,
//         act == GripperAction::OPEN ? "OPEN" : "CLOSE");
//       request_gripper(act, /*on_done=*/nullptr); // 논블로킹 호출
//     }
//   }

//   void maybe_finish() {
//     if (rb3_done_all_ && ur3_done_all_) {
//       RCLCPP_INFO(this->get_logger(), "[%s] === Sequence ALL DONE ===", ts().c_str());
//     }
//   }

//   // 상태 콜백
//   void on_right_status(GoalStatusArray::SharedPtr msg)
//   {
//     const bool active = any_active(*msg);
//     if (rb3_waiting_) {
//       if (active && !rb3_seen_active_) {
//         rb3_seen_active_ = true;
//         t_active_rb3_ = std::chrono::steady_clock::now();
//         auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_rb3_ - t_send_rb3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB3] FIRST ACTIVE (Δsend→active=%lldms)", ts().c_str(), (long long)d);
//       }
//       if (rb3_seen_active_ && !active) {
//         rb3_waiting_ = false;
//         auto now = std::chrono::steady_clock::now();
//         auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_rb3_).count();
//         auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_rb3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB3] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
//                     ts().c_str(), idx_rb3_, (long long)d1, (long long)d2);

//         if (sync_inflight_.has_value() && idx_rb3_ == sync_inflight_->first) {
//           run_after_hook_barrier_only(/*is_left=*/false, idx_rb3_);  // ★FIX
//           rb3_inflight_done_ = true;
//           check_inflight_and_advance_after_both_done();
//           return;
//         }
//         if (handle_sync_reached(/*is_left=*/false, idx_rb3_)) return;

//         advance_rb3_after_done(idx_rb3_);
//       }
//     }
//   }
//   void on_left_status(GoalStatusArray::SharedPtr msg)
//   {
//     const bool active = any_active(*msg);
//     if (ur3_waiting_) {
//       if (active && !ur3_seen_active_) {
//         ur3_seen_active_ = true;
//         t_active_ur3_ = std::chrono::steady_clock::now();
//         auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_ur3_ - t_send_ur3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] FIRST ACTIVE (Δsend→active=%lldms)", ts().c_str(), (long long)d);
//       }
//       if (ur3_seen_active_ && !active) {
//         ur3_waiting_ = false;
//         auto now = std::chrono::steady_clock::now();
//         auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_ur3_).count();
//         auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_ur3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
//                     ts().c_str(), idx_ur3_, (long long)d1, (long long)d2);

//         if (sync_inflight_.has_value() && idx_ur3_ == sync_inflight_->second) {
//           run_after_hook_barrier_only(/*is_left=*/true, idx_ur3_);   // ★FIX (여기서 step3 OPEN 실행)
//           ur3_inflight_done_ = true;
//           check_inflight_and_advance_after_both_done();
//           return;
//         }
//         if (handle_sync_reached(/*is_left=*/true, idx_ur3_)) return;

//         advance_ur3_after_done(idx_ur3_);
//       }
//     }
//   }

//   bool handle_sync_reached(bool is_left, size_t just_finished_idx)
//   {
//     bool matched_any = false;

//     for (auto &sp : sync_points_) {
//       if (sp.fired) continue;

//       if (!is_left && just_finished_idx == sp.rb3_idx) {
//         sp.reached_rb3 = true; matched_any = true;
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] RB3 reached idx=%zu", ts().c_str(), sp.rb3_idx);
//       }
//       if ( is_left && just_finished_idx == sp.ur3_idx) {
//         sp.reached_ur3 = true; matched_any = true;
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] UR3e reached idx=%zu", ts().c_str(), sp.ur3_idx);
//       }

//       if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
//         sp.fired = true;

//         idx_rb3_ = sp.rb3_next;
//         idx_ur3_ = sp.ur3_next;

//         sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//         rb3_inflight_done_ = false;
//         ur3_inflight_done_ = false;

//         RCLCPP_INFO(this->get_logger(),
//           "[%s] [SYNC] BOTH reached → FIRE NOW (RB3=%zu, UR3e=%zu)",
//           ts().c_str(), idx_rb3_, idx_ur3_);

//         fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
//         return true;
//       }
//     }
//     return matched_any;
//   }

//   void check_inflight_and_advance_after_both_done()
//   {
//     if (!sync_inflight_.has_value()) return;
//     if (!(rb3_inflight_done_ && ur3_inflight_done_)) return;

//     auto [rb3_idx, ur3_idx] = *sync_inflight_;
//     RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] inflight done (RB3=%zu, UR3e=%zu)", ts().c_str(), rb3_idx, ur3_idx);

//     sync_inflight_.reset();
//     rb3_inflight_done_ = ur3_inflight_done_ = false;

//     ++idx_rb3_;  start_rb3_step(idx_rb3_);
//     ++idx_ur3_;  start_ur3_step(idx_ur3_);
//   }

//   void request_gripper(GripperAction action, std::function<void()> on_done)
//   {
//     if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
//     auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
//     auto cli = (action == GripperAction::CLOSE) ? cli_grip_close_ : cli_grip_open_;
//     if (!cli->wait_for_service(0s)) {
//       RCLCPP_WARN(this->get_logger(), "[%s] Gripper service not available (%s).",
//                   ts().c_str(), action == GripperAction::CLOSE ? "CLOSE" : "OPEN");
//       if (on_done) on_done();
//       return;
//     }
//     auto t0 = std::chrono::steady_clock::now();
//     cli->async_send_request(
//       req,
//       [this, action, on_done, t0](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
//         bool ok = false;
//         try {
//           auto resp = future.get();
//           ok = resp && resp->success;
//           auto now = std::chrono::steady_clock::now();
//           auto d = std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
//           RCLCPP_INFO(this->get_logger(), "[%s] [GRIPPER %s] %s (Δcall=%lldms)",
//             ts().c_str(),
//             (action == GripperAction::CLOSE ? "CLOSE" : "OPEN"),
//             ok ? "OK" : "FAILED",
//             (long long)d);
//         } catch (...) {
//           RCLCPP_ERROR(this->get_logger(), "[%s] [GRIPPER] exception while calling service", ts().c_str());
//         }
//         if (on_done) on_done();
//       });
//   }
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MoveSequenceClient>());
//   rclcpp::shutdown();
//   return 0;
// }













//===========================핸드 추가 버전=====================
// #include <memory>
// #include <vector>
// #include <string>
// #include <chrono>
// #include <unordered_map>
// #include <functional>
// #include <optional>
// #include <atomic>
// #include <sstream>
// #include <iomanip>

// #include <rclcpp/rclcpp.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <action_msgs/msg/goal_status_array.hpp>

// #include "dual_arm_msg/srv/movetcppos_rb3.hpp"
// #include "dual_arm_msg/srv/movetcppos_ur3.hpp"

// // ★ RB(오른팔)용 Inspire Hand 서비스
// #include "inspire_hand_interface/srv/setangle.hpp"

// using namespace std::chrono_literals;
// using action_msgs::msg::GoalStatusArray;
// using action_msgs::msg::GoalStatus;

// using RB3Srv = dual_arm_msg::srv::MovetcpposRB3;
// using UR3Srv = dual_arm_msg::srv::MovetcpposUR3;
// using HandSetAngleSrv = inspire_hand_interface::srv::Setangle;

// struct Pose6 { double x, y, z, rx, ry, rz; };

// enum class GripperAction { NONE = 0, OPEN, CLOSE, PINCH };

// class MoveSequenceClient : public rclcpp::Node
// {
// public:
//   MoveSequenceClient()
//   : Node("move_sequence_client_event_driven")
//   , t0_steady_(std::chrono::steady_clock::now())
//   {
//     right_fjt_base_ = declare_parameter<std::string>(
//       "right_fjt_base", "/joint_trajectory_controller/follow_joint_trajectory");
//     left_fjt_base_ = declare_parameter<std::string>(
//       "left_fjt_base", "/scaled_joint_trajectory_controller/follow_joint_trajectory");

//     cli_rb3_ = this->create_client<RB3Srv>("zmk_tcppos_rb3");
//     cli_ur3_ = this->create_client<UR3Srv>("zmk_tcppos_ur3e");

//     // UR(왼팔) 그리퍼(기존 Trigger) — 그대로 유지
//     cli_grip_open_  = this->create_client<std_srvs::srv::Trigger>("zmk_girp_open");
//     cli_grip_close_ = this->create_client<std_srvs::srv::Trigger>("zmk_girp_close");

//     // ★ RB(오른팔) Inspire Hand 서비스 클라이언트 (절대 경로)
//     hand_service_name_ = declare_parameter<std::string>("hand_service_name", "/Setangle");
//     cli_hand_setangle_ = this->create_client<HandSetAngleSrv>(hand_service_name_);

//     // 시퀀스
//     seq_rb3_ = {
//       {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (0)
//       {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (1)
//       {-0.04642, -0.44152, 0.11987, 1.76208441,  0.24801129,  1.77238186}, // (2)
//       { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (3)
//       { 0.14883, -0.37773, 0.30176, 0.1439897,   0.1534144,   1.4289011},  // (4)
//       { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (5)
//       {-0.04381, -0.45377, 0.13585, 1.75492856,  0.19338248,  1.8102555},  // (6)
//       {-0.0407,  -0.44568, 0.31162, 1.7228145,   0.17715092,  1.80205245}, // (7)
//       {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (8)
//     };
//     seq_ur3_ = {
//       { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)
//       { -0.140,   -0.410,   0.165,   0.0,   -3.14,  0.0 },      // (1)
//       { -0.13056, -0.32030, 0.46642, 2.249,  2.489, -2.523 },   // (2)
//       { -0.188,   -0.32030, 0.46642, 2.249,  2.489, -2.523 },   // (3)
//       { -0.13056, -0.32030, 0.46642, 2.249,  2.489, -2.523 },   // (4)
//       { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (5)
//     };

//     // 그리퍼/핸드 훅: UR은 그대로, RB는 Inspire Hand
//     ur3_grip_before_[0] = GripperAction::OPEN;
//     ur3_grip_after_[1]  = GripperAction::CLOSE;
//     ur3_grip_after_[3]  = GripperAction::OPEN;

//     // RB after 훅으로 Hand 조작 (요청하신 것)
//     rb3_grip_after_[0]  = GripperAction::OPEN;
//     rb3_grip_before_[2] = GripperAction::PINCH;
//     rb3_grip_after_[2]  = GripperAction::CLOSE;
//     rb3_grip_before_[3]  = GripperAction::CLOSE;
//     rb3_grip_after_[6]  = GripperAction::PINCH;
//     rb3_grip_after_[7]  = GripperAction::OPEN;

//     // 배리어(동기점)
//     sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/3});
//     // sync_points_.push_back(PairSync{/*rb3_idx=*/4, /*ur3_idx=*/3, /*rb3_next=*/5, /*ur3_next=*/4});

//     start_timer_ = this->create_wall_timer(300ms, [this] {
//       if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s) ||
//           !cli_grip_open_->wait_for_service(0s) || !cli_grip_close_->wait_for_service(0s) ||
//           !cli_hand_setangle_->wait_for_service(0s)) {
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
//           "[%s] Waiting for services (rb3/ur3e/gripper/hand). hand_service=%s",
//           ts().c_str(), hand_service_name_.c_str());
//         return;
//       }
//       const auto right_status = resolve_status_topic(right_fjt_base_);
//       const auto left_status  = resolve_status_topic(left_fjt_base_);
//       if (right_status.empty() || left_status.empty()) {
//         RCLCPP_ERROR(get_logger(),
//           "[%s] Could not resolve action status topics. right_base=%s left_base=%s",
//           ts().c_str(), right_fjt_base_.c_str(), left_fjt_base_.c_str());
//         return;
//       }
//       RCLCPP_INFO(get_logger(), "[%s] [RESOLVED] RIGHT status: %s", ts().c_str(), right_status.c_str());
//       RCLCPP_INFO(get_logger(), "[%s] [RESOLVED] LEFT  status: %s", ts().c_str(), left_status.c_str());

//       sub_right_status_ = this->create_subscription<GoalStatusArray>(
//         right_status, rclcpp::QoS(50),
//         std::bind(&MoveSequenceClient::on_right_status, this, std::placeholders::_1));
//       sub_left_status_ = this->create_subscription<GoalStatusArray>(
//         left_status, rclcpp::QoS(50),
//         std::bind(&MoveSequenceClient::on_left_status, this, std::placeholders::_1));

//       RCLCPP_INFO(get_logger(), "[%s] Services ready. Using hand_service=%s",
//                   ts().c_str(), hand_service_name_.c_str());

//       start_timer_->cancel();
//       start_sequence();
//     });
//   }

// private:
//   struct PairSync {
//     size_t rb3_idx;
//     size_t ur3_idx;
//     size_t rb3_next;
//     size_t ur3_next;
//     bool reached_rb3 = false;
//     bool reached_ur3 = false;
//     bool fired = false;
//   };
//   std::vector<PairSync> sync_points_;

//   std::optional<std::pair<size_t,size_t>> sync_inflight_;
//   bool rb3_inflight_done_ = false;
//   bool ur3_inflight_done_ = false;

//   rclcpp::Client<RB3Srv>::SharedPtr cli_rb3_;
//   rclcpp::Client<UR3Srv>::SharedPtr cli_ur3_;

//   // UR(왼팔) 그리퍼 트리거
//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_open_;
//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_close_;

//   // ★ RB(오른팔) Inspire Hand
//   rclcpp::Client<HandSetAngleSrv>::SharedPtr cli_hand_setangle_;
//   std::string hand_service_name_;

//   rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_right_status_;
//   rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_left_status_;
//   rclcpp::TimerBase::SharedPtr start_timer_;
//   std::string right_fjt_base_, left_fjt_base_;
//   std::vector<Pose6> seq_rb3_, seq_ur3_;
//   size_t idx_rb3_ = 0, idx_ur3_ = 0;
//   bool rb3_waiting_ = false, ur3_waiting_ = false;
//   bool rb3_seen_active_ = false, ur3_seen_active_ = false;
//   bool rb3_done_all_ = false,  ur3_done_all_ = false;

//   std::unordered_map<size_t, GripperAction> ur3_grip_before_;
//   std::unordered_map<size_t, GripperAction> ur3_grip_after_;
//   std::unordered_map<size_t, GripperAction> rb3_grip_before_;
//   std::unordered_map<size_t, GripperAction> rb3_grip_after_;

//   rclcpp::TimerBase::SharedPtr sync_fire_timer_;

//   std::chrono::steady_clock::time_point t0_steady_;
//   std::chrono::steady_clock::time_point t_send_rb3_, t_send_ur3_;
//   std::chrono::steady_clock::time_point t_active_rb3_, t_active_ur3_;

//   std::string ts() const {
//     using namespace std::chrono;
//     auto now = steady_clock::now();
//     auto ms  = duration_cast<milliseconds>(now - t0_steady_).count();
//     std::ostringstream oss; oss << std::setw(9) << ms << "ms";
//     return oss.str();
//   }
//   static bool ends_with(const std::string &s, const std::string &suffix) {
//     return s.size() >= suffix.size() &&
//            s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
//   }
//   static bool any_active(const GoalStatusArray &arr) {
//     for (const auto &st : arr.status_list) {
//       if (st.status == GoalStatus::STATUS_ACCEPTED ||
//           st.status == GoalStatus::STATUS_EXECUTING ||
//           st.status == GoalStatus::STATUS_CANCELING) return true;
//     }
//     return false;
//   }

//   std::string resolve_status_topic(const std::string &base)
//   {
//     const auto a = base + "/_action/status";
//     const auto b = base + "/status";
//     auto graph = this->get_topic_names_and_types();
//     if (graph.find(a) != graph.end()) return a;
//     if (graph.find(b) != graph.end()) return b;
//     for (const auto &kv : graph) {
//       const auto &topic = kv.first;
//       if ((ends_with(topic, "/_action/status") || ends_with(topic, "/status")) &&
//           topic.find(base) != std::string::npos) {
//         return topic;
//       }
//     }
//     return std::string();
//   }

//   void start_sequence() {
//     RCLCPP_INFO(this->get_logger(), "[%s] === Sequence PREP ===", ts().c_str());
//     prepare_and_fire_first_pair();
//   }

//   void prepare_and_fire_first_pair() {
//     const bool need_rb3 = rb3_grip_before_.count(0) && rb3_grip_before_[0] != GripperAction::NONE;
//     const bool need_ur3 = ur3_grip_before_.count(0) && ur3_grip_before_[0] != GripperAction::NONE;

//     if (!need_rb3 && !need_ur3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] No BEFORE hooks at idx0 → FIRE BOTH step0 now", ts().c_str());
//       sync_inflight_ = std::make_pair(0u, 0u);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;
//       fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
//       return;
//     }

//     RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks at idx0: RB3=%s, UR3=%s",
//       ts().c_str(), need_rb3 ? "YES" : "NO", need_ur3 ? "YES" : "NO");

//     auto pending = std::make_shared<std::atomic<int>>(0);
//     if (need_rb3) pending->fetch_add(1);
//     if (need_ur3) pending->fetch_add(1);

//     auto after = [this, pending]() {
//       if (pending->fetch_sub(1) == 1) {
//         RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks done → FIRE BOTH step0", ts().c_str());
//         sync_inflight_ = std::make_pair(0u, 0u);
//         rb3_inflight_done_ = ur3_inflight_done_ = false;
//         fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
//       }
//     };

//     if (need_rb3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [PREP] RB3 BEFORE hook (idx0)", ts().c_str());
//       request_rb_hand(rb3_grip_before_[0], after);
//     }
//     if (need_ur3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [PREP] UR3e BEFORE hook (idx0)", ts().c_str());
//       request_ur_gripper(ur3_grip_before_[0], after);
//     }
//   }

//   void fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks)
//   {
//     if (!apply_before_hooks) {
//       sync_fire_timer_ = this->create_wall_timer(
//         0ms,
//         [this, rb3_idx, ur3_idx](){
//           RCLCPP_INFO(this->get_logger(),
//             "[%s] FIRE BOTH NOW (RB3=%zu, UR3e=%zu) — same tick",
//             ts().c_str(), rb3_idx, ur3_idx);
//           send_ur3_step(ur3_idx);
//           send_rb3_step(rb3_idx);
//           sync_fire_timer_->cancel();
//         }
//       );
//       return;
//     }

//     size_t pending = 0;
//     bool do_rb3_before = rb3_grip_before_.count(rb3_idx) &&
//                         rb3_grip_before_[rb3_idx] != GripperAction::NONE;
//     bool do_ur3_before = ur3_grip_before_.count(ur3_idx) &&
//                         ur3_grip_before_[ur3_idx] != GripperAction::NONE;

//     auto after_one_done = [&]() {
//       if (--pending == 0) {
//         sync_fire_timer_ = this->create_wall_timer(
//           0ms,
//           [this, rb3_idx, ur3_idx](){
//             RCLCPP_INFO(this->get_logger(),
//               "[%s] FIRE BOTH (after hooks) (RB3=%zu, UR3e=%zu)", ts().c_str(), rb3_idx, ur3_idx);
//             send_rb3_step(rb3_idx);
//             send_ur3_step(ur3_idx);
//             sync_fire_timer_->cancel();
//           }
//         );
//       }
//     };

//     if (do_rb3_before) pending++;
//     if (do_ur3_before) pending++;
//     if (pending == 0) {
//       sync_fire_timer_ = this->create_wall_timer(
//         0ms, [this, rb3_idx, ur3_idx](){
//           RCLCPP_INFO(this->get_logger(),
//             "[%s] FIRE BOTH (no hooks) (RB3=%zu, UR3e=%zu)", ts().c_str(), rb3_idx, ur3_idx);
//           send_rb3_step(rb3_idx);
//           send_ur3_step(ur3_idx);
//           sync_fire_timer_->cancel();
//         }
//       );
//       return;
//     }
//     if (do_rb3_before) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] RB3 BEFORE hook (idx%zu)", ts().c_str(), rb3_idx);
//       request_rb_hand(rb3_grip_before_[rb3_idx], [after_one_done]{});
//     }
//     if (do_ur3_before) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] UR3e BEFORE hook (idx%zu)", ts().c_str(), ur3_idx);
//       request_ur_gripper(ur3_grip_before_[ur3_idx], [after_one_done]{});
//     }
//   }

//   // ===== RB3 =====
//   void start_rb3_step(size_t i)
//   {
//     if (i >= seq_rb3_.size()) { if (!rb3_done_all_) { rb3_done_all_ = true; maybe_finish(); } return; }
//     if (rb3_grip_before_.count(i) && rb3_grip_before_[i] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB3] BEFORE hook at idx=%zu", ts().c_str(), i);
//       // request_rb_hand(rb3_grip_before_[i], [this, i]{ send_rb3_step(i); });
//       request_rb_hand(rb3_grip_before_[i], /*on_done=*/nullptr);
//       send_rb3_step(i);
//     } else {
//       send_rb3_step(i);
//     }
//   }
//   void send_rb3_step(size_t i)
//   {
//     if (i >= seq_rb3_.size()) return;
//     const auto &p = seq_rb3_[i];
//     auto req = std::make_shared<RB3Srv::Request>();
//     req->right_x = p.x; req->right_y = p.y; req->right_z = p.z;
//     req->right_rx = p.rx; req->right_ry = p.ry; req->right_rz = p.rz;

//     rb3_waiting_ = true;
//     rb3_seen_active_ = false;
//     t_send_rb3_ = std::chrono::steady_clock::now();

//     RCLCPP_INFO(this->get_logger(), "[%s] [RB3] SEND step %zu", ts().c_str(), i);
//     cli_rb3_->async_send_request(
//       req,
//       [this, i](rclcpp::Client<RB3Srv>::SharedFuture future) {
//         auto resp = future.get();
//         if (resp && resp->success) {
//           RCLCPP_INFO(this->get_logger(), "[%s] [RB3] step %zu SENT OK: %s",
//                       ts().c_str(), i, resp->message.c_str());
//         } else {
//           RCLCPP_ERROR(this->get_logger(), "[%s] [RB3] step %zu SEND FAILED", ts().c_str(), i);
//         }
//       });
//   }
//   void advance_rb3_after_done(size_t just_finished_idx)
//   {
//     if (rb3_grip_after_.count(just_finished_idx) &&
//         rb3_grip_after_[just_finished_idx] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB3] AFTER hook at idx=%zu", ts().c_str(), just_finished_idx);
//       request_rb_hand(rb3_grip_after_[just_finished_idx], [this]{ ++idx_rb3_; start_rb3_step(idx_rb3_); });
//     } else {
//       ++idx_rb3_;
//       start_rb3_step(idx_rb3_);
//     }
//   }

//   // ===== UR3e =====
//   void start_ur3_step(size_t i)
//   {
//     if (i >= seq_ur3_.size()) { if (!ur3_done_all_) { ur3_done_all_ = true; maybe_finish(); } return; }
//     if (ur3_grip_before_.count(i) && ur3_grip_before_[i] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] BEFORE hook at idx=%zu", ts().c_str(), i);
//       request_ur_gripper(ur3_grip_before_[i], [this, i]{ send_ur3_step(i); });
//     } else {
//       send_ur3_step(i);
//     }
//   }
//   void send_ur3_step(size_t i)
//   {
//     if (i >= seq_ur3_.size()) return;
//     const auto &p = seq_ur3_[i];
//     auto req = std::make_shared<UR3Srv::Request>();
//     req->left_x = p.x; req->left_y = p.y; req->left_z = p.z;
//     req->left_rx = p.rx; req->left_ry = p.ry; req->left_rz = p.rz;

//     ur3_waiting_ = true;
//     ur3_seen_active_ = false;
//     t_send_ur3_ = std::chrono::steady_clock::now();

//     RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] SEND step %zu", ts().c_str(), i);
//     cli_ur3_->async_send_request(
//       req,
//       [this, i](rclcpp::Client<UR3Srv>::SharedFuture future) {
//         auto resp = future.get();
//         if (resp && resp->success) {
//           RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] step %zu SENT OK%s",
//                       ts().c_str(), i, resp->message.c_str());
//         } else {
//           RCLCPP_ERROR(this->get_logger(), "[%s] [UR3e] step %zu SEND FAILED", ts().c_str(), i);
//         }
//       });
//   }
//   void advance_ur3_after_done(size_t just_finished_idx)
//   {
//     if (ur3_grip_after_.count(just_finished_idx) &&
//         ur3_grip_after_[just_finished_idx] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] AFTER hook at idx=%zu", ts().c_str(), just_finished_idx);
//       request_ur_gripper(ur3_grip_after_[just_finished_idx], [this]{ ++idx_ur3_; start_ur3_step(idx_ur3_); });
//     } else {
//       ++idx_ur3_;
//       start_ur3_step(idx_ur3_);
//     }
//   }

//   // ★ inflight(배리어)로 보낸 스텝이 끝났을 때도 AFTER 훅 수행
//   void run_after_hook_barrier_only(bool is_left, size_t finished_idx)
//   {
//     GripperAction act = GripperAction::NONE;
//     if (is_left) {
//       auto it = ur3_grip_after_.find(finished_idx);
//       if (it != ur3_grip_after_.end()) act = it->second;
//     } else {
//       auto it = rb3_grip_after_.find(finished_idx);
//       if (it != rb3_grip_after_.end()) act = it->second;
//     }
//     if (act != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(),
//         "[%s] [BARRIER AFTER] %s idx=%zu → %s",
//         ts().c_str(), is_left ? "UR3e" : "RB3", finished_idx,
//         act == GripperAction::OPEN ? "OPEN" : "CLOSE");
//       if (is_left) request_ur_gripper(act, /*on_done=*/nullptr);
//       else         request_rb_hand(act,    /*on_done=*/nullptr);
//     }
//   }

//   void maybe_finish() {
//     if (rb3_done_all_ && ur3_done_all_) {
//       RCLCPP_INFO(this->get_logger(), "[%s] === Sequence ALL DONE ===", ts().c_str());
//     }
//   }

//   // 상태 콜백
//   void on_right_status(GoalStatusArray::SharedPtr msg)
//   {
//     const bool active = any_active(*msg);
//     if (rb3_waiting_) {
//       if (active && !rb3_seen_active_) {
//         rb3_seen_active_ = true;
//         t_active_rb3_ = std::chrono::steady_clock::now();
//         auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_rb3_ - t_send_rb3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB3] FIRST ACTIVE (Δsend→active=%lldms)", ts().c_str(), (long long)d);
//       }
//       if (rb3_seen_active_ && !active) {
//         rb3_waiting_ = false;
//         auto now = std::chrono::steady_clock::now();
//         auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_rb3_).count();
//         auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_rb3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB3] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
//                     ts().c_str(), idx_rb3_, (long long)d1, (long long)d2);

//         if (sync_inflight_.has_value() && idx_rb3_ == sync_inflight_->first) {
//           run_after_hook_barrier_only(/*is_left=*/false, idx_rb3_);  // inflight after
//           rb3_inflight_done_ = true;
//           check_inflight_and_advance_after_both_done();
//           return;
//         }
//         if (handle_sync_reached(/*is_left=*/false, idx_rb3_)) return;

//         advance_rb3_after_done(idx_rb3_);
//       }
//     }
//   }
//   void on_left_status(GoalStatusArray::SharedPtr msg)
//   {
//     const bool active = any_active(*msg);
//     if (ur3_waiting_) {
//       if (active && !ur3_seen_active_) {
//         ur3_seen_active_ = true;
//         t_active_ur3_ = std::chrono::steady_clock::now();
//         auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_ur3_ - t_send_ur3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] FIRST ACTIVE (Δsend→active=%lldms)", ts().c_str(), (long long)d);
//       }
//       if (ur3_seen_active_ && !active) {
//         ur3_waiting_ = false;
//         auto now = std::chrono::steady_clock::now();
//         auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_ur3_).count();
//         auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_ur3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
//                     ts().c_str(), idx_ur3_, (long long)d1, (long long)d2);

//         if (sync_inflight_.has_value() && idx_ur3_ == sync_inflight_->second) {
//           run_after_hook_barrier_only(/*is_left=*/true, idx_ur3_);   // inflight after
//           ur3_inflight_done_ = true;
//           check_inflight_and_advance_after_both_done();
//           return;
//         }
//         if (handle_sync_reached(/*is_left=*/true, idx_ur3_)) return;

//         advance_ur3_after_done(idx_ur3_);
//       }
//     }
//   }

//   bool handle_sync_reached(bool is_left, size_t just_finished_idx)
//   {
//     bool matched_any = false;

//     for (auto &sp : sync_points_) {
//       if (sp.fired) continue;

//       if (!is_left && just_finished_idx == sp.rb3_idx) {
//         sp.reached_rb3 = true; matched_any = true;
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] RB3 reached idx=%zu", ts().c_str(), sp.rb3_idx);
//         // ★ NEW: 동기점 트리거 인덱스에서도 AFTER 훅 실행(논블로킹)
//         run_after_hook_barrier_only(/*is_left=*/false, just_finished_idx);
//       }
//       if ( is_left && just_finished_idx == sp.ur3_idx) {
//         sp.reached_ur3 = true; matched_any = true;
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] UR3e reached idx=%zu", ts().c_str(), sp.ur3_idx);
//         // ★ NEW: 동기점 트리거 인덱스에서도 AFTER 훅 실행(논블로킹)
//         run_after_hook_barrier_only(/*is_left=*/true, just_finished_idx);
//       }

//       if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
//         sp.fired = true;

//         idx_rb3_ = sp.rb3_next;
//         idx_ur3_ = sp.ur3_next;

//         sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//         rb3_inflight_done_ = false;
//         ur3_inflight_done_ = false;

//         RCLCPP_INFO(this->get_logger(),
//           "[%s] [SYNC] BOTH reached → FIRE NOW (RB3=%zu, UR3e=%zu)",
//           ts().c_str(), idx_rb3_, idx_ur3_);

//         fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
//         return true;
//       }
//     }
//     return matched_any;
//   }

//   void check_inflight_and_advance_after_both_done()
//   {
//     if (!sync_inflight_.has_value()) return;
//     if (!(rb3_inflight_done_ && ur3_inflight_done_)) return;

//     auto [rb3_idx, ur3_idx] = *sync_inflight_;
//     RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] inflight done (RB3=%zu, UR3e=%zu)", ts().c_str(), rb3_idx, ur3_idx);

//     sync_inflight_.reset();
//     rb3_inflight_done_ = ur3_inflight_done_ = false;

//     ++idx_rb3_;  start_rb3_step(idx_rb3_);
//     ++idx_ur3_;  start_ur3_step(idx_ur3_);
//   }

//   // === UR(왼팔) 그리퍼(Trigger) ===
//   void request_ur_gripper(GripperAction action, std::function<void()> on_done)
//   {
//     if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
//     auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
//     auto cli = (action == GripperAction::CLOSE) ? cli_grip_close_ : cli_grip_open_;
//     if (!cli->wait_for_service(0s)) {
//       RCLCPP_WARN(this->get_logger(), "[%s] UR gripper service not available (%s).",
//                   ts().c_str(), action == GripperAction::CLOSE ? "CLOSE" : "OPEN");
//       if (on_done) on_done();
//       return;
//     }
//     auto t0 = std::chrono::steady_clock::now();
//     cli->async_send_request(
//       req,
//       [this, action, on_done, t0](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
//         bool ok = false;
//         try { auto resp = future.get(); ok = resp && resp->success; } catch (...) {}
//         auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
//                     std::chrono::steady_clock::now() - t0).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR %s] %s (Δcall=%lldms)",
//           ts().c_str(), (action == GripperAction::CLOSE ? "CLOSE" : "OPEN"), ok ? "OK" : "FAILED", (long long)dt);
//         if (on_done) on_done();
//       });
//   }

//   // === RB(오른팔) Inspire Hand (/Setangle) ===
//   void request_rb_hand(GripperAction action, std::function<void()> on_done)
//   {
//     if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
//     if (!cli_hand_setangle_->wait_for_service(0s)) {
//       RCLCPP_WARN(this->get_logger(), "[%s] RB hand service not available: %s",
//                   ts().c_str(), hand_service_name_.c_str());
//       if (on_done) on_done();
//       return;
//     }

//     auto req = std::make_shared<HandSetAngleSrv::Request>();
//     // 고정 프리셋 (요청하신 CLI 값과 동일)
//     if (action == GripperAction::CLOSE)
//     {
//       // req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
//       // req->angle3 = 100;  req->angle4 = 350;  req->angle5 = 0;
//       req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 645;
//       req->angle3 = 671;  req->angle4 = 260;  req->angle5 = 0;
//     }
//     else if(action == GripperAction::PINCH)
//     {
//       req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
//       req->angle3 = 1000; req->angle4 = 400; req->angle5 = 0;
//     }
//     else
//     { // OPEN
//       req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
//       req->angle3 = 1000; req->angle4 = 1000; req->angle5 = 0;
//     }
//     req->hand_id = 1;
//     req->status  = "set_angle";
    
//     const char* tag =
//        (action == GripperAction::CLOSE ? "CLOSE" :
//        action == GripperAction::OPEN  ? "OPEN"  : "PINCH");

//     // const char* tag = (action == GripperAction::CLOSE ? "CLOSE" : "OPEN");
//     auto t0 = std::chrono::steady_clock::now();
//     cli_hand_setangle_->async_send_request(
//       req,
//       [this, on_done, t0, tag](rclcpp::Client<HandSetAngleSrv>::SharedFuture future) {
//         bool ok = false;
//         try {
//           auto resp = future.get();
//           ok = (resp != nullptr); // 서버 응답만 확인(패키지별 필드 차이 대응)
//         } catch (...) {}
//         auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
//                     std::chrono::steady_clock::now() - t0).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB HAND %s] %s (Δcall=%lldms)",
//           ts().c_str(), tag, ok ? "CALLED" : "FAILED", (long long)dt);
//         if (on_done) on_done();
//       }
//     );
//   }
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MoveSequenceClient>());
//   rclcpp::shutdown();
//   return 0;
// }



//===========================bimanual 동기화 추가========================

// #include <memory>
// #include <vector>
// #include <string>
// #include <chrono>
// #include <unordered_map>
// #include <functional>
// #include <optional>
// #include <atomic>
// #include <sstream>
// #include <iomanip>

// #include <rclcpp/rclcpp.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <action_msgs/msg/goal_status_array.hpp>

// #include "dual_arm_msg/srv/movetcppos_rb3.hpp"
// #include "dual_arm_msg/srv/movetcppos_ur3.hpp"
// #include "dual_arm_msg/srv/movetcppos.hpp"
// #include "inspire_hand_interface/srv/setangle.hpp"

// using namespace std::chrono_literals;
// using action_msgs::msg::GoalStatusArray;
// using action_msgs::msg::GoalStatus;

// using RB3Srv = dual_arm_msg::srv::MovetcpposRB3;
// using UR3Srv = dual_arm_msg::srv::MovetcpposUR3;
// using HandSetAngleSrv = inspire_hand_interface::srv::Setangle;

// struct Pose6 { double x, y, z, rx, ry, rz; };

// enum class GripperAction { NONE = 0, OPEN, CLOSE, PINCH };

// class MoveSequenceClient : public rclcpp::Node
// {
// public:
//   MoveSequenceClient()
//   : Node("move_sequence_client_event_driven")
//   , t0_steady_(std::chrono::steady_clock::now())
//   {

//     exit_when_done_ = this->declare_parameter<bool>("exit_when_done", true);
    
//     right_fjt_base_ = declare_parameter<std::string>(
//       "right_fjt_base", "/joint_trajectory_controller/follow_joint_trajectory");
//     left_fjt_base_ = declare_parameter<std::string>(
//       "left_fjt_base", "/scaled_joint_trajectory_controller/follow_joint_trajectory");

//     cli_rb3_ = this->create_client<RB3Srv>("zmk_tcppos_rb3");
//     cli_ur3_ = this->create_client<UR3Srv>("zmk_tcppos_ur3e");

//     // UR 그리퍼
//     cli_grip_open_  = this->create_client<std_srvs::srv::Trigger>("zmk_girp_open");
//     cli_grip_close_ = this->create_client<std_srvs::srv::Trigger>("zmk_girp_close");

//     // RB Inspire Hand
//     hand_service_name_ = declare_parameter<std::string>("hand_service_name", "/Setangle");
//     cli_hand_setangle_ = this->create_client<HandSetAngleSrv>(hand_service_name_);

//     // 양팔 동시 bimanual 서비스
//     bimanual_srv_name_ = declare_parameter<std::string>(
//       "bimanual_srv_name", "zmk_move_to_tcppos");
//     use_bimanual_srv_ = declare_parameter<bool>("use_bimanual_srv", true);
//     client_bimanual_tcppos_ = this->create_client<dual_arm_msg::srv::Movetcppos>(bimanual_srv_name_);

//     // 시퀀스
//     seq_rb3_ = {
//       {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (0)
//       {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (1)
//       {-0.04642, -0.44152, 0.11987, 1.76208441,  0.24801129,  1.77238186}, // (2)
//       { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (3) // 결합전
//       { 0.07944, -0.22198, 0.41277, 1.84917634,  -1.4379768,  1.4158111}, // (4) // 카메라 앞
//       { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (5) // 결합전
//       { 0.135,   -0.36851, 0.30087, 0.1446878,   0.1537635,   1.322436},   // (6) // 결합
//       { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (7)
//       {-0.04381, -0.45377, 0.13585, 1.75492856,  0.19338248,  1.8102555},  // (8)
//       {-0.0407,  -0.44568, 0.31162, 1.7228145,   0.17715092,  1.80205245}, // (9)
//       {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (10)
//     };
//     seq_ur3_ = {
//       { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)
//       { -0.140,   -0.410,   0.175,   0.0,   -3.14,  0.0 },      // (1)
//       { -0.0800,  -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (2)
//       { -0.13056, -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (3)
//       { -0.185,   -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (4)
//       { -0.13056, -0.32030, 0.46642, 2.151,  0.109, -2.257 },   // (5)
//       { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (6)
//     };

//     // 그리퍼/핸드 훅
//     ur3_grip_before_[0] = GripperAction::OPEN;
//     ur3_grip_after_[1]  = GripperAction::CLOSE;
//     ur3_grip_after_[4]  = GripperAction::OPEN;

//     rb3_grip_after_[0]  = GripperAction::OPEN;
//     rb3_grip_before_[2] = GripperAction::PINCH;
//     rb3_grip_after_[2]  = GripperAction::CLOSE;
//     rb3_grip_before_[3] = GripperAction::CLOSE;
//     rb3_grip_after_[8]  = GripperAction::PINCH;
//     rb3_grip_after_[9]  = GripperAction::OPEN;

//     // 배리어(동기점)
//     sync_points_.push_back(PairSync{/*rb3_idx=*/5, /*ur3_idx=*/3, /*rb3_next=*/6, /*ur3_next=*/4});
//     sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/2});

//     start_timer_ = this->create_wall_timer(300ms, [this] {
//       if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s) ||
//           !cli_grip_open_->wait_for_service(0s) || !cli_grip_close_->wait_for_service(0s) ||
//           !cli_hand_setangle_->wait_for_service(0s) ||
//           (use_bimanual_srv_ && !client_bimanual_tcppos_->wait_for_service(0s))) {
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
//           "[%s] Waiting for services (rb3/ur3e/gripper/hand%s). hand_service=%s, bimanual=%s",
//           ts().c_str(), use_bimanual_srv_ ? "/bimanual" : "",
//           hand_service_name_.c_str(), bimanual_srv_name_.c_str());
//         return;
//       }
//       const auto right_status = resolve_status_topic(right_fjt_base_);
//       const auto left_status  = resolve_status_topic(left_fjt_base_);
//       if (right_status.empty() || left_status.empty()) {
//         RCLCPP_ERROR(get_logger(),
//           "[%s] Could not resolve action status topics. right_base=%s left_base=%s",
//           ts().c_str(), right_fjt_base_.c_str(), left_fjt_base_.c_str());
//         return;
//       }
//       RCLCPP_INFO(get_logger(), "[%s] [RESOLVED] RIGHT status: %s", ts().c_str(), right_status.c_str());
//       RCLCPP_INFO(get_logger(), "[%s] [RESOLVED] LEFT  status: %s", ts().c_str(), left_status.c_str());

//       sub_right_status_ = this->create_subscription<GoalStatusArray>(
//         right_status, rclcpp::QoS(50),
//         std::bind(&MoveSequenceClient::on_right_status, this, std::placeholders::_1));
//       sub_left_status_ = this->create_subscription<GoalStatusArray>(
//         left_status, rclcpp::QoS(50),
//         std::bind(&MoveSequenceClient::on_left_status, this, std::placeholders::_1));

//       RCLCPP_INFO(get_logger(), "[%s] Services ready. Using hand_service=%s, bimanual=%s (use=%s)",
//                   ts().c_str(), hand_service_name_.c_str(), bimanual_srv_name_.c_str(),
//                   use_bimanual_srv_ ? "true" : "false");

//       start_timer_->cancel();
//       start_sequence();
//     });
//   }

// private:
//   struct PairSync {
//     size_t rb3_idx;
//     size_t ur3_idx;
//     size_t rb3_next;
//     size_t ur3_next;
//     bool reached_rb3 = false;
//     bool reached_ur3 = false;
//     bool fired = false;
//   };
//   std::vector<PairSync> sync_points_;

//   std::optional<std::pair<size_t,size_t>> sync_inflight_;
//   bool rb3_inflight_done_ = false;
//   bool ur3_inflight_done_ = false;

//   rclcpp::Client<RB3Srv>::SharedPtr cli_rb3_;
//   rclcpp::Client<UR3Srv>::SharedPtr cli_ur3_;

//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_open_;
//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_close_;

//   rclcpp::Client<HandSetAngleSrv>::SharedPtr cli_hand_setangle_;
//   std::string hand_service_name_;

//   rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedPtr client_bimanual_tcppos_;
//   std::string bimanual_srv_name_;
//   bool use_bimanual_srv_ = true;

//   rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_right_status_;
//   rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_left_status_;
//   rclcpp::TimerBase::SharedPtr start_timer_;
//   std::string right_fjt_base_, left_fjt_base_;
//   std::vector<Pose6> seq_rb3_, seq_ur3_;
//   size_t idx_rb3_ = 0, idx_ur3_ = 0;
//   bool rb3_waiting_ = false, ur3_waiting_ = false;
//   bool rb3_seen_active_ = false, ur3_seen_active_ = false;
//   bool rb3_done_all_ = false,  ur3_done_all_ = false;

//   std::unordered_map<size_t, GripperAction> ur3_grip_before_;
//   std::unordered_map<size_t, GripperAction> ur3_grip_after_;
//   std::unordered_map<size_t, GripperAction> rb3_grip_before_;
//   std::unordered_map<size_t, GripperAction> rb3_grip_after_;

//   rclcpp::TimerBase::SharedPtr sync_fire_timer_;

//   std::chrono::steady_clock::time_point t0_steady_;
//   std::chrono::steady_clock::time_point t_send_rb3_, t_send_ur3_;
//   std::chrono::steady_clock::time_point t_active_rb3_, t_active_ur3_;

//   bool exit_when_done_{true};
//   bool shutdown_scheduled_{false};
//   rclcpp::TimerBase::SharedPtr shutdown_timer_;

//   std::string ts() const {
//     using namespace std::chrono;
//     auto now = steady_clock::now();
//     auto ms  = duration_cast<milliseconds>(now - t0_steady_).count();
//     std::ostringstream oss; oss << std::setw(9) << ms << "ms";
//     return oss.str();
//   }
//   static bool ends_with(const std::string &s, const std::string &suffix) {
//     return s.size() >= suffix.size() &&
//            s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
//   }
//   static bool any_active(const GoalStatusArray &arr) {
//     for (const auto &st : arr.status_list) {
//       if (st.status == GoalStatus::STATUS_ACCEPTED ||
//           st.status == GoalStatus::STATUS_EXECUTING ||
//           st.status == GoalStatus::STATUS_CANCELING) return true;
//     }
//     return false;
//   }

//   std::string resolve_status_topic(const std::string &base)
//   {
//     const auto a = base + "/_action/status";
//     const auto b = base + "/status";
//     auto graph = this->get_topic_names_and_types();
//     if (graph.find(a) != graph.end()) return a;
//     if (graph.find(b) != graph.end()) return b;
//     for (const auto &kv : graph) {
//       const auto &topic = kv.first;
//       if ((ends_with(topic, "/_action/status") || ends_with(topic, "/status")) &&
//           topic.find(base) != std::string::npos) {
//         return topic;
//       }
//     }
//     return std::string();
//   }

//   void start_sequence() {
//     RCLCPP_INFO(this->get_logger(), "[%s] === Sequence PREP ===", ts().c_str());
//     prepare_and_fire_first_pair();
//   }

//   void prepare_and_fire_first_pair() {
//     const bool need_rb3 = rb3_grip_before_.count(0) && rb3_grip_before_[0] != GripperAction::NONE;
//     const bool need_ur3 = ur3_grip_before_.count(0) && ur3_grip_before_[0] != GripperAction::NONE;

//     if (!need_rb3 && !need_ur3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] No BEFORE hooks at idx0 → FIRE BOTH step0 now", ts().c_str());
//       sync_inflight_ = std::make_pair(0u, 0u);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;
//       fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
//       return;
//     }

//     RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks at idx0: RB3=%s, UR3=%s",
//       ts().c_str(), need_rb3 ? "YES" : "NO", need_ur3 ? "YES" : "NO");

//     auto pending = std::make_shared<std::atomic<int>>(0);
//     if (need_rb3) pending->fetch_add(1);
//     if (need_ur3) pending->fetch_add(1);

//     auto after = [this, pending]() {
//       if (pending->fetch_sub(1) == 1) {
//         RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks done → FIRE BOTH step0", ts().c_str());
//         sync_inflight_ = std::make_pair(0u, 0u);
//         rb3_inflight_done_ = ur3_inflight_done_ = false;
//         fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
//       }
//     };

//     if (need_rb3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [PREP] RB3 BEFORE hook (idx0)", ts().c_str());
//       request_rb_hand(rb3_grip_before_[0], after);
//     }
//     if (need_ur3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [PREP] UR3e BEFORE hook (idx0)", ts().c_str());
//       request_ur_gripper(ur3_grip_before_[0], after);
//     }
//   }

//   void fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks)
//   {
//     if (!apply_before_hooks) {
//       sync_fire_timer_ = this->create_wall_timer(
//         0ms,
//         [this, rb3_idx, ur3_idx](){
//           RCLCPP_INFO(this->get_logger(),
//             "[%s] FIRE BOTH NOW (RB3=%zu, UR3e=%zu) — same tick",
//             ts().c_str(), rb3_idx, ur3_idx);

//           if (use_bimanual_srv_) {
//             const auto &r = seq_rb3_[rb3_idx];
//             const auto &l = seq_ur3_[ur3_idx];
//             call_zmk_move_to_bimanual_tcppos_async(
//               /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
//               /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
//           } else {
//             send_ur3_step(ur3_idx);
//             send_rb3_step(rb3_idx);
//           }
//           sync_fire_timer_->cancel();
//         }
//       );
//       return;
//     }

//     size_t pending = 0;
//     bool do_rb3_before = rb3_grip_before_.count(rb3_idx) &&
//                         rb3_grip_before_[rb3_idx] != GripperAction::NONE;
//     bool do_ur3_before = ur3_grip_before_.count(ur3_idx) &&
//                         ur3_grip_before_[ur3_idx] != GripperAction::NONE;

//     auto after_one_done = [&]() {
//       if (--pending == 0) {
//         sync_fire_timer_ = this->create_wall_timer(
//           0ms,
//           [this, rb3_idx, ur3_idx](){
//             RCLCPP_INFO(this->get_logger(),
//               "[%s] FIRE BOTH (after hooks) (RB3=%zu, UR3e=%zu)", ts().c_str(), rb3_idx, ur3_idx);

//             if (use_bimanual_srv_) {
//               const auto &r = seq_rb3_[rb3_idx];
//               const auto &l = seq_ur3_[ur3_idx];
//               call_zmk_move_to_bimanual_tcppos_async(
//                 /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
//                 /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
//             } else {
//               send_rb3_step(rb3_idx);
//               send_ur3_step(ur3_idx);
//             }
//             sync_fire_timer_->cancel();
//           }
//         );
//       }
//     };

//     if (do_rb3_before) pending++;
//     if (do_ur3_before) pending++;
//     if (pending == 0) {
//       sync_fire_timer_ = this->create_wall_timer(
//         0ms, [this, rb3_idx, ur3_idx](){
//           RCLCPP_INFO(this->get_logger(),
//             "[%s] FIRE BOTH (no hooks) (RB3=%zu, UR3e=%zu)", ts().c_str(), rb3_idx, ur3_idx);
//           if (use_bimanual_srv_) {
//             const auto &r = seq_rb3_[rb3_idx];
//             const auto &l = seq_ur3_[ur3_idx];
//             call_zmk_move_to_bimanual_tcppos_async(
//               /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
//               /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
//           } else {
//             send_rb3_step(rb3_idx);
//             send_ur3_step(ur3_idx);
//           }
//           sync_fire_timer_->cancel();
//         }
//       );
//       return;
//     }
//     if (do_rb3_before) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] RB3 BEFORE hook (idx%zu)", ts().c_str(), rb3_idx);
//       request_rb_hand(rb3_grip_before_[rb3_idx], [after_one_done](){});
//     }
//     if (do_ur3_before) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] UR3e BEFORE hook (idx%zu)", ts().c_str(), ur3_idx);
//       request_ur_gripper(ur3_grip_before_[ur3_idx], [after_one_done](){});
//     }
//   }

//   // ===== (동기) 양팔 동시 tcppos 호출 ====
//   void call_zmk_move_to_bimanual_tcppos(
//       double l_x, double l_y, double l_z, double l_rx, double l_ry, double l_rz,
//       double r_x, double r_y, double r_z, double r_rx, double r_ry, double r_rz)
//   {
//     auto req = std::make_shared<dual_arm_msg::srv::Movetcppos::Request>();
//     // 왼팔(UR3e)
//     req->left_x = l_x;  req->left_y = l_y;  req->left_z = l_z;
//     req->left_rx = l_rx; req->left_ry = l_ry; req->left_rz = l_rz;
//     // 오른팔(RB3)
//     req->right_x = r_x; req->right_y = r_y; req->right_z = r_z;
//     req->right_rx = r_rx; req->right_ry = r_ry; req->right_rz = r_rz;

//     if (!client_bimanual_tcppos_->wait_for_service(std::chrono::seconds(2))) {
//       RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] service not available: %s", bimanual_srv_name_.c_str());
//       return;
//     }

//     auto future = client_bimanual_tcppos_->async_send_request(req);
//     auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5));

//     if (ret != rclcpp::FutureReturnCode::SUCCESS || !future.valid()) {
//       RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] service timeout or invalid future");
//       return;
//     }

//     const auto resp = future.get();  // 반드시 1회만!
//     if (resp && resp->success) {
//       RCLCPP_INFO(this->get_logger(), "[BIMANUAL] move_tcppos success");
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] move_tcppos failed: %s",
//                    resp ? resp->message.c_str() : "no response");
//     }
//   }

//   // ===== (비동기) 상태머신 친화 래퍼 ====
//   void call_zmk_move_to_bimanual_tcppos_async(
//       double l_x, double l_y, double l_z, double l_rx, double l_ry, double l_rz,
//       double r_x, double r_y, double r_z, double r_rx, double r_ry, double r_rz)
//   {
//     auto req = std::make_shared<dual_arm_msg::srv::Movetcppos::Request>();
//     // 왼팔(UR3e)
//     req->left_x = l_x;  req->left_y = l_y;  req->left_z = l_z;
//     req->left_rx = l_rx; req->left_ry = l_ry; req->left_rz = l_rz;
//     // 오른팔(RB3)
//     req->right_x = r_x; req->right_y = r_y; req->right_z = r_z;
//     req->right_rx = r_rx; req->right_ry = r_ry; req->right_rz = r_rz;

//     // 상태머신 트리거
//     rb3_waiting_ = ur3_waiting_ = true;
//     rb3_seen_active_ = ur3_seen_active_ = false;
//     t_send_rb3_ = t_send_ur3_ = std::chrono::steady_clock::now();

//     RCLCPP_INFO(this->get_logger(),
//       "[%s] [BIMANUAL] SEND tcppos (UR3e & RB3) via %s", ts().c_str(), bimanual_srv_name_.c_str());

//     client_bimanual_tcppos_->async_send_request(
//       req,
//       [this](rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedFuture f){
//         bool ok = false; std::string msg = "no response";
//         try { auto resp = f.get(); ok = resp && resp->success; if (resp) msg = resp->message; }
//         catch (const std::exception& e) { msg = e.what(); }
//         catch (...) {}
//         RCLCPP_INFO(this->get_logger(), "[%s] [BIMANUAL] request SENT %s (%s)",
//                     ts().c_str(), ok ? "OK" : "FAILED", msg.c_str());
//       });
//   }

//   // ===== RB3 =====
//   void start_rb3_step(size_t i)
//   {
//     if (i >= seq_rb3_.size()) { if (!rb3_done_all_) { rb3_done_all_ = true; maybe_finish(); } return; }
//     if (rb3_grip_before_.count(i) && rb3_grip_before_[i] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB3] BEFORE hook at idx=%zu", ts().c_str(), i);
//       // request_rb_hand(rb3_grip_before_[i], [this, i]{ send_rb3_step(i); });
//       request_rb_hand(rb3_grip_before_[i], /*on_done=*/nullptr);
//       send_rb3_step(i);
//     } else {
//       send_rb3_step(i);
//     }
//   }
//   void send_rb3_step(size_t i)
//   {
//     if (i >= seq_rb3_.size()) return;
//     const auto &p = seq_rb3_[i];
//     auto req = std::make_shared<RB3Srv::Request>();
//     req->right_x = p.x; req->right_y = p.y; req->right_z = p.z;
//     req->right_rx = p.rx; req->right_ry = p.ry; req->right_rz = p.rz;

//     rb3_waiting_ = true;
//     rb3_seen_active_ = false;
//     t_send_rb3_ = std::chrono::steady_clock::now();

//     RCLCPP_INFO(this->get_logger(), "[%s] [RB3] SEND step %zu", ts().c_str(), i);
//     cli_rb3_->async_send_request(
//       req,
//       [this, i](rclcpp::Client<RB3Srv>::SharedFuture future) {
//         auto resp = future.get();
//         if (resp && resp->success) {
//           RCLCPP_INFO(this->get_logger(), "[%s] [RB3] step %zu SENT OK: %s",
//                       ts().c_str(), i, resp->message.c_str());
//         } else {
//           RCLCPP_ERROR(this->get_logger(), "[%s] [RB3] step %zu SEND FAILED", ts().c_str(), i);
//         }
//       });
//   }
//   void advance_rb3_after_done(size_t just_finished_idx)
//   {
//     if (rb3_grip_after_.count(just_finished_idx) &&
//         rb3_grip_after_[just_finished_idx] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB3] AFTER hook at idx=%zu", ts().c_str(), just_finished_idx);
//       request_rb_hand(rb3_grip_after_[just_finished_idx], [this]{ ++idx_rb3_; start_rb3_step(idx_rb3_); });
//     } else {
//       ++idx_rb3_;
//       start_rb3_step(idx_rb3_);
//     }
//   }

//   // ===== UR3e =====
//   void start_ur3_step(size_t i)
//   {
//     if (i >= seq_ur3_.size()) { if (!ur3_done_all_) { ur3_done_all_ = true; maybe_finish(); } return; }
//     if (ur3_grip_before_.count(i) && ur3_grip_before_[i] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] BEFORE hook at idx=%zu", ts().c_str(), i);
//       request_ur_gripper(ur3_grip_before_[i], [this, i]{ send_ur3_step(i); });
//     } else {
//       send_ur3_step(i);
//     }
//   }
//   void send_ur3_step(size_t i)
//   {
//     if (i >= seq_ur3_.size()) return;
//     const auto &p = seq_ur3_[i];
//     auto req = std::make_shared<UR3Srv::Request>();
//     req->left_x = p.x; req->left_y = p.y; req->left_z = p.z;
//     req->left_rx = p.rx; req->left_ry = p.ry; req->left_rz = p.rz;

//     ur3_waiting_ = true;
//     ur3_seen_active_ = false;
//     t_send_ur3_ = std::chrono::steady_clock::now();

//     RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] SEND step %zu", ts().c_str(), i);
//     cli_ur3_->async_send_request(
//       req,
//       [this, i](rclcpp::Client<UR3Srv>::SharedFuture future) {
//         auto resp = future.get();
//         if (resp && resp->success) {
//           RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] step %zu SENT OK: %s",
//                       ts().c_str(), i, resp->message.c_str());
//         } else {
//           RCLCPP_ERROR(this->get_logger(), "[%s] [UR3e] step %zu SEND FAILED", ts().c_str(), i);
//         }
//       });
//   }
//   void advance_ur3_after_done(size_t just_finished_idx)
//   {
//     if (ur3_grip_after_.count(just_finished_idx) &&
//         ur3_grip_after_[just_finished_idx] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] AFTER hook at idx=%zu", ts().c_str(), just_finished_idx);
//       request_ur_gripper(ur3_grip_after_[just_finished_idx], [this]{ ++idx_ur3_; start_ur3_step(idx_ur3_); });
//     } else {
//       ++idx_ur3_;
//       start_ur3_step(idx_ur3_);
//     }
//   }

//   void run_after_hook_barrier_only(bool is_left, size_t finished_idx)
//   {
//     GripperAction act = GripperAction::NONE;
//     if (is_left) {
//       auto it = ur3_grip_after_.find(finished_idx);
//       if (it != ur3_grip_after_.end()) act = it->second;
//     } else {
//       auto it = rb3_grip_after_.find(finished_idx);
//       if (it != rb3_grip_after_.end()) act = it->second;
//     }
//     if (act != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(),
//         "[%s] [BARRIER AFTER] %s idx=%zu → %s",
//         ts().c_str(), is_left ? "UR3e" : "RB3", finished_idx,
//         act == GripperAction::OPEN ? "OPEN" : (act == GripperAction::CLOSE ? "CLOSE" : "PINCH"));
//       if (is_left) request_ur_gripper(act, /*on_done=*/nullptr);
//       else         request_rb_hand(act,    /*on_done=*/nullptr);
//     }
//   }

//   void maybe_finish() {
//     if (rb3_done_all_ && ur3_done_all_) {
//       RCLCPP_INFO(this->get_logger(), "[%s] === Sequence ALL DONE ===", ts().c_str());

//       if (exit_when_done_ && !shutdown_scheduled_) {
//         shutdown_scheduled_ = true;
//         // 로그 flush/액션 정리 약간의 여유
//         shutdown_timer_ = this->create_wall_timer(
//           std::chrono::milliseconds(200),
//           [this]() {
//             RCLCPP_INFO(this->get_logger(), "[%s] Shutting down (exit_when_done=true).", ts().c_str());
//             rclcpp::shutdown();
//           });
//       }
//     }
//   }

//   // 상태 콜백
//   void on_right_status(GoalStatusArray::SharedPtr msg)
//   {
//     const bool active = any_active(*msg);
//     if (rb3_waiting_) {
//       if (active && !rb3_seen_active_) {
//         rb3_seen_active_ = true;
//         t_active_rb3_ = std::chrono::steady_clock::now();
//         auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_rb3_ - t_send_rb3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB3] FIRST ACTIVE (Δsend→active=%lldms)", ts().c_str(), (long long)d);
//       }
//       if (rb3_seen_active_ && !active) {
//         rb3_waiting_ = false;
//         auto now = std::chrono::steady_clock::now();
//         auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_rb3_).count();
//         auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_rb3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB3] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
//                     ts().c_str(), idx_rb3_, (long long)d1, (long long)d2);

//         if (sync_inflight_.has_value() && idx_rb3_ == sync_inflight_->first) {
//           run_after_hook_barrier_only(/*is_left=*/false, idx_rb3_);  // inflight after
//           rb3_inflight_done_ = true;
//           check_inflight_and_advance_after_both_done();
//           return;
//         }
//         if (handle_sync_reached(/*is_left=*/false, idx_rb3_)) return;

//         advance_rb3_after_done(idx_rb3_);
//       }
//     }
//   }
//   void on_left_status(GoalStatusArray::SharedPtr msg)
//   {
//     const bool active = any_active(*msg);
//     if (ur3_waiting_) {
//       if (active && !ur3_seen_active_) {
//         ur3_seen_active_ = true;
//         t_active_ur3_ = std::chrono::steady_clock::now();
//         auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_ur3_ - t_send_ur3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] FIRST ACTIVE (Δsend→active=%lldms)", ts().c_str(), (long long)d);
//       }
//       if (ur3_seen_active_ && !active) {
//         ur3_waiting_ = false;
//         auto now = std::chrono::steady_clock::now();
//         auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_ur3_).count();
//         auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_ur3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
//                     ts().c_str(), idx_ur3_, (long long)d1, (long long)d2);

//         if (sync_inflight_.has_value() && idx_ur3_ == sync_inflight_->second) {
//           run_after_hook_barrier_only(/*is_left=*/true, idx_ur3_);   // inflight after
//           ur3_inflight_done_ = true;
//           check_inflight_and_advance_after_both_done();
//           return;
//         }
//         if (handle_sync_reached(/*is_left=*/true, idx_ur3_)) return;

//         advance_ur3_after_done(idx_ur3_);
//       }
//     }
//   }

//   bool handle_sync_reached(bool is_left, size_t just_finished_idx)
//   {
//     bool matched_any = false;

//     for (auto &sp : sync_points_) {
//       if (sp.fired) continue;

//       if (!is_left && just_finished_idx == sp.rb3_idx) {
//         sp.reached_rb3 = true; matched_any = true;
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] RB3 reached idx=%zu", ts().c_str(), sp.rb3_idx);
//         run_after_hook_barrier_only(/*is_left=*/false, just_finished_idx);
//       }
//       if ( is_left && just_finished_idx == sp.ur3_idx) {
//         sp.reached_ur3 = true; matched_any = true;
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] UR3e reached idx=%zu", ts().c_str(), sp.ur3_idx);
//         run_after_hook_barrier_only(/*is_left=*/true, just_finished_idx);
//       }

//       if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
//         sp.fired = true;

//         idx_rb3_ = sp.rb3_next;
//         idx_ur3_ = sp.ur3_next;

//         sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//         rb3_inflight_done_ = false;
//         ur3_inflight_done_ = false;

//         RCLCPP_INFO(this->get_logger(),
//           "[%s] [SYNC] BOTH reached → FIRE NOW (RB3=%zu, UR3e=%zu)",
//           ts().c_str(), idx_rb3_, idx_ur3_);

//         fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
//         return true;
//       }
//     }
//     return matched_any;
//   }

//   void check_inflight_and_advance_after_both_done()
//   {
//     if (!sync_inflight_.has_value()) return;
//     if (!(rb3_inflight_done_ && ur3_inflight_done_)) return;

//     auto [rb3_idx, ur3_idx] = *sync_inflight_;
//     RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] inflight done (RB3=%zu, UR3e=%zu)", ts().c_str(), rb3_idx, ur3_idx);

//     sync_inflight_.reset();
//     rb3_inflight_done_ = ur3_inflight_done_ = false;

//     ++idx_rb3_;  start_rb3_step(idx_rb3_);
//     ++idx_ur3_;  start_ur3_step(idx_ur3_);
//   }

//   // === UR(왼팔) 그리퍼(Trigger) ===
//   void request_ur_gripper(GripperAction action, std::function<void()> on_done)
//   {
//     if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
//     auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
//     auto cli = (action == GripperAction::CLOSE) ? cli_grip_close_ : cli_grip_open_;
//     if (!cli->wait_for_service(0s)) {
//       RCLCPP_WARN(this->get_logger(), "[%s] UR gripper service not available (%s).",
//                   ts().c_str(), action == GripperAction::CLOSE ? "CLOSE" : "OPEN");
//       if (on_done) on_done();
//       return;
//     }
//     auto t0 = std::chrono::steady_clock::now();
//     cli->async_send_request(
//       req,
//       [this, action, on_done, t0](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
//         bool ok = false;
//         try { auto resp = future.get(); ok = resp && resp->success; } catch (...) {}
//         auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
//                     std::chrono::steady_clock::now() - t0).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR %s] %s (Δcall=%lldms)",
//           ts().c_str(), (action == GripperAction::CLOSE ? "CLOSE" : "OPEN"), ok ? "OK" : "FAILED", (long long)dt);
//         if (on_done) on_done();
//       });
//   }

//   // === RB(오른팔) Inspire Hand (/Setangle) ===
//   void request_rb_hand(GripperAction action, std::function<void()> on_done)
//   {
//     if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
//     if (!cli_hand_setangle_->wait_for_service(0s)) {
//       RCLCPP_WARN(this->get_logger(), "[%s] RB hand service not available: %s",
//                   ts().c_str(), hand_service_name_.c_str());
//       if (on_done) on_done();
//       return;
//     }

//     auto req = std::make_shared<HandSetAngleSrv::Request>();
//     // 고정 프리셋
//     if (action == GripperAction::CLOSE)
//     {
//       req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 645;
//       req->angle3 = 671;  req->angle4 = 260;  req->angle5 = 0;
//     }
//     else if(action == GripperAction::PINCH)
//     {
//       req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
//       req->angle3 = 1000; req->angle4 = 400;  req->angle5 = 0;
//     }
//     else
//     { // OPEN
//       req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
//       req->angle3 = 1000; req->angle4 = 1000; req->angle5 = 0;
//     }
//     req->hand_id = 1;
//     req->status  = "set_angle";
    
//     const char* tag =
//        (action == GripperAction::CLOSE ? "CLOSE" :
//        action == GripperAction::OPEN  ? "OPEN"  : "PINCH");

//     auto t0 = std::chrono::steady_clock::now();
//     cli_hand_setangle_->async_send_request(
//       req,
//       [this, on_done, t0, tag](rclcpp::Client<HandSetAngleSrv>::SharedFuture future) {
//         bool ok = false;
//         try {
//           auto resp = future.get();
//           ok = (resp != nullptr);
//         } catch (...) {}
//         auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
//                     std::chrono::steady_clock::now() - t0).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB HAND %s] %s (Δcall=%lldms)",
//           ts().c_str(), tag, ok ? "CALLED" : "FAILED", (long long)dt);
//         if (on_done) on_done();
//       }
//     );
//   }
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MoveSequenceClient>());
//   rclcpp::shutdown();
//   return 0;
// }








// --------------중간에 카메라 보는거 추가 및 중간 서비스 추가 및 ur wrist 90도 회전--------------------

// #include <memory>
// #include <vector>
// #include <string>
// #include <chrono>
// #include <unordered_map>
// #include <functional>
// #include <optional>
// #include <atomic>
// #include <sstream>
// #include <iomanip>

// #include <rclcpp/rclcpp.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <action_msgs/msg/goal_status_array.hpp>

// #include "dual_arm_msg/srv/movetcppos_rb3.hpp"
// #include "dual_arm_msg/srv/movetcppos_ur3.hpp"
// #include "dual_arm_msg/srv/movetcppos.hpp"
// #include "inspire_hand_interface/srv/setangle.hpp"

// using namespace std::chrono_literals;
// using action_msgs::msg::GoalStatusArray;
// using action_msgs::msg::GoalStatus;

// using RB3Srv = dual_arm_msg::srv::MovetcpposRB3;
// using UR3Srv = dual_arm_msg::srv::MovetcpposUR3;
// using HandSetAngleSrv = inspire_hand_interface::srv::Setangle;

// struct Pose6 { double x, y, z, rx, ry, rz; };

// enum class GripperAction { NONE = 0, OPEN, CLOSE, PINCH };

// class MoveSequenceClient : public rclcpp::Node
// {
// public:
//   MoveSequenceClient()
//   : Node("move_sequence_client_event_driven")
//   , t0_steady_(std::chrono::steady_clock::now())
//   {
//     exit_when_done_ = this->declare_parameter<bool>("exit_when_done", true);

//     right_fjt_base_ = declare_parameter<std::string>(
//       "right_fjt_base", "/joint_trajectory_controller/follow_joint_trajectory");
//     left_fjt_base_ = declare_parameter<std::string>(
//       "left_fjt_base", "/scaled_joint_trajectory_controller/follow_joint_trajectory");

//     cli_rb3_ = this->create_client<RB3Srv>("zmk_tcppos_rb3");
//     cli_ur3_ = this->create_client<UR3Srv>("zmk_tcppos_ur3e");

//     // UR 그리퍼
//     cli_grip_open_  = this->create_client<std_srvs::srv::Trigger>("zmk_girp_open");
//     cli_grip_close_ = this->create_client<std_srvs::srv::Trigger>("zmk_girp_close");

//     // RB Inspire Hand
//     hand_service_name_ = declare_parameter<std::string>("hand_service_name", "/Setangle");
//     cli_hand_setangle_ = this->create_client<HandSetAngleSrv>(hand_service_name_);

//     // 양팔 동시 bimanual 서비스
//     bimanual_srv_name_ = declare_parameter<std::string>(
//       "bimanual_srv_name", "zmk_move_to_tcppos");
//     use_bimanual_srv_ = this->declare_parameter<bool>("use_bimanual_srv", true);
//     client_bimanual_tcppos_ = this->create_client<dual_arm_msg::srv::Movetcppos>(bimanual_srv_name_);

//     // ==== Degcheck용 대기/해제 서비스 서버 ====
//     srv_degcheck_ = this->create_service<std_srvs::srv::Trigger>(
//       "zmk_DegcheckFlag",
//       [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
//              std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
//       {
//         if (!hold_until_degcheck_) {
//           resp->success = false;
//           resp->message = "Not holding at (RB3=4, UR3=2).";
//           return;
//         }

//         hold_until_degcheck_ = false;

//         RCLCPP_INFO(this->get_logger(), "[%s] [DEGCHECK/SERVER] Release → FIRE (RB3=5, UR3=3)",
//                     ts().c_str());

//         idx_rb3_ = 5;
//         idx_ur3_ = 3;

//         // 인플라이트 세팅
//         sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//         rb3_inflight_done_ = ur3_inflight_done_ = false;

//         // 즉시 동시 발사
//         fire_both_immediate(idx_rb3_, idx_ur3_);

//         resp->success = true;
//         resp->message = "Proceed fired to (RB3=5, UR3=3).";
//       }
//     );

//     // 시퀀스
//     seq_rb3_ = {
//       {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (0)
//       {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (1)
//       {-0.04642, -0.44152, 0.11987, 1.76208441,  0.24801129,  1.77238186}, // (2)
//       { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (3) // 결합전
//       { 0.07944, -0.22198, 0.41277, 1.84917634,  -1.4379768,  1.4158111}, // (4) // 카메라 앞
//       { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (5) // 결합전
//       { 0.135,   -0.36851, 0.30087, 0.1446878,   0.1537635,   1.322436},   // (6) // 결합
//       { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (7)
//       {-0.04381, -0.45377, 0.13585, 1.75492856,  0.19338248,  1.8102555},  // (8)
//       {-0.0407,  -0.44568, 0.31162, 1.7228145,   0.17715092,  1.80205245}, // (9)
//       {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (10)
//     };
//     seq_ur3_ = {
//       { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)
//       { -0.140,   -0.410,   0.175,   0.0,   -3.14,  0.0 },      // (1)
//       { -0.0800,  -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (2)
//       { -0.13056, -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (3)
//       { -0.185,   -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (4)
//       { -0.13056, -0.32030, 0.46642, 2.151,  0.109, -2.257 },   // (5)
//       { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (6)
//     };

//     // 그리퍼/핸드 훅
//     ur3_grip_before_[0] = GripperAction::OPEN;
//     ur3_grip_after_[1]  = GripperAction::CLOSE;
//     ur3_grip_after_[4]  = GripperAction::OPEN;

//     rb3_grip_after_[0]  = GripperAction::OPEN;
//     rb3_grip_before_[2] = GripperAction::PINCH;
//     rb3_grip_after_[2]  = GripperAction::CLOSE;
//     rb3_grip_before_[3] = GripperAction::CLOSE;
//     rb3_grip_after_[8]  = GripperAction::PINCH;
//     rb3_grip_after_[9]  = GripperAction::OPEN;

//     // 배리어(동기점)
//     sync_points_.push_back(PairSync{/*rb3_idx=*/5, /*ur3_idx=*/3, /*rb3_next=*/6, /*ur3_next=*/4});
//     sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/2});

//     start_timer_ = this->create_wall_timer(300ms, [this] {
//       if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s) ||
//           !cli_grip_open_->wait_for_service(0s) || !cli_grip_close_->wait_for_service(0s) ||
//           !cli_hand_setangle_->wait_for_service(0s) ||
//           (use_bimanual_srv_ && !client_bimanual_tcppos_->wait_for_service(0s))) {
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
//           "[%s] Waiting for services (rb3/ur3e/gripper/hand%s). hand_service=%s, bimanual=%s",
//           ts().c_str(), use_bimanual_srv_ ? "/bimanual" : "",
//           hand_service_name_.c_str(), bimanual_srv_name_.c_str());
//         return;
//       }
//       const auto right_status = resolve_status_topic(right_fjt_base_);
//       const auto left_status  = resolve_status_topic(left_fjt_base_);
//       if (right_status.empty() || left_status.empty()) {
//         RCLCPP_ERROR(get_logger(),
//           "[%s] Could not resolve action status topics. right_base=%s left_base=%s",
//           ts().c_str(), right_fjt_base_.c_str(), left_fjt_base_.c_str());
//         return;
//       }
//       RCLCPP_INFO(get_logger(), "[%s] [RESOLVED] RIGHT status: %s", ts().c_str(), right_status.c_str());
//       RCLCPP_INFO(get_logger(), "[%s] [RESOLVED] LEFT  status: %s", ts().c_str(), left_status.c_str());

//       sub_right_status_ = this->create_subscription<GoalStatusArray>(
//         right_status, rclcpp::QoS(50),
//         std::bind(&MoveSequenceClient::on_right_status, this, std::placeholders::_1));
//       sub_left_status_ = this->create_subscription<GoalStatusArray>(
//         left_status, rclcpp::QoS(50),
//         std::bind(&MoveSequenceClient::on_left_status, this, std::placeholders::_1));

//       RCLCPP_INFO(get_logger(), "[%s] Services ready. Using hand_service=%s, bimanual=%s (use=%s)",
//                   ts().c_str(), hand_service_name_.c_str(), bimanual_srv_name_.c_str(),
//                   use_bimanual_srv_ ? "true" : "false");

//       start_timer_->cancel();
//       start_sequence();
//     });
//   }

// private:
//   struct PairSync {
//     size_t rb3_idx;
//     size_t ur3_idx;
//     size_t rb3_next;
//     size_t ur3_next;
//     bool reached_rb3 = false;
//     bool reached_ur3 = false;
//     bool fired = false;
//   };
//   std::vector<PairSync> sync_points_;

//   std::optional<std::pair<size_t,size_t>> sync_inflight_;
//   bool rb3_inflight_done_ = false;
//   bool ur3_inflight_done_ = false;

//   rclcpp::Client<RB3Srv>::SharedPtr cli_rb3_;
//   rclcpp::Client<UR3Srv>::SharedPtr cli_ur3_;

//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_open_;
//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_close_;

//   rclcpp::Client<HandSetAngleSrv>::SharedPtr cli_hand_setangle_;
//   std::string hand_service_name_;

//   rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedPtr client_bimanual_tcppos_;
//   std::string bimanual_srv_name_;
//   bool use_bimanual_srv_ = true;

//   // Degcheck 서버 & 대기 플래그
//   rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_degcheck_;
//   bool hold_until_degcheck_{false};

//   rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_right_status_;
//   rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_left_status_;
//   rclcpp::TimerBase::SharedPtr start_timer_;
//   std::string right_fjt_base_, left_fjt_base_;
//   std::vector<Pose6> seq_rb3_, seq_ur3_;
//   size_t idx_rb3_ = 0, idx_ur3_ = 0;
//   bool rb3_waiting_ = false, ur3_waiting_ = false;
//   bool rb3_seen_active_ = false, ur3_seen_active_ = false;
//   bool rb3_done_all_ = false,  ur3_done_all_ = false;

//   std::unordered_map<size_t, GripperAction> ur3_grip_before_;
//   std::unordered_map<size_t, GripperAction> ur3_grip_after_;
//   std::unordered_map<size_t, GripperAction> rb3_grip_before_;
//   std::unordered_map<size_t, GripperAction> rb3_grip_after_;

//   // (기존 0ms 타이머 제거 방향으로 사용 안 함)
//   rclcpp::TimerBase::SharedPtr sync_fire_timer_;

//   std::chrono::steady_clock::time_point t0_steady_;
//   std::chrono::steady_clock::time_point t_send_rb3_, t_send_ur3_;
//   std::chrono::steady_clock::time_point t_active_rb3_, t_active_ur3_;

//   bool exit_when_done_{true};
//   bool shutdown_scheduled_{false};
//   rclcpp::TimerBase::SharedPtr shutdown_timer_;

//   std::string ts() const {
//     using namespace std::chrono;
//     auto now = steady_clock::now();
//     auto ms  = duration_cast<std::chrono::milliseconds>(now - t0_steady_).count();
//     std::ostringstream oss; oss << std::setw(9) << ms << "ms";
//     return oss.str();
//   }
//   static bool ends_with(const std::string &s, const std::string &suffix) {
//     if (suffix.size() > s.size()) return false;
//     return s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
//   }
//   static bool any_active(const GoalStatusArray &arr) {
//     for (const auto &st : arr.status_list) {
//       if (st.status == GoalStatus::STATUS_ACCEPTED ||
//           st.status == GoalStatus::STATUS_EXECUTING ||
//           st.status == GoalStatus::STATUS_CANCELING) return true;
//     }
//     return false;
//   }

//   std::string resolve_status_topic(const std::string &base)
//   {
//     const auto a = base + "/_action/status";
//     const auto b = base + "/status";
//     auto graph = this->get_topic_names_and_types();
//     if (graph.find(a) != graph.end()) return a;
//     if (graph.find(b) != graph.end()) return b;
//     for (const auto &kv : graph) {
//       const auto &topic = kv.first;
//       if ((ends_with(topic, "/_action/status") || ends_with(topic, "/status")) &&
//           topic.find(base) != std::string::npos) {
//         return topic;
//       }
//     }
//     return std::string();
//   }

//   void start_sequence() {
//     RCLCPP_INFO(this->get_logger(), "[%s] === Sequence PREP ===", ts().c_str());
//     prepare_and_fire_first_pair();
//   }

//   void prepare_and_fire_first_pair() {
//     const bool need_rb3 = rb3_grip_before_.count(0) && rb3_grip_before_[0] != GripperAction::NONE;
//     const bool need_ur3 = ur3_grip_before_.count(0) && ur3_grip_before_[0] != GripperAction::NONE;

//     if (!need_rb3 && !need_ur3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] No BEFORE hooks at idx0 → FIRE BOTH step0 now", ts().c_str());
//       sync_inflight_ = std::make_pair(0u, 0u);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;
//       // 동시 발사 즉시 호출
//       fire_both_immediate(/*rb3_idx=*/0, /*ur3_idx=*/0);
//       return;
//     }

//     RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks at idx0: RB3=%s, UR3=%s",
//       ts().c_str(), need_rb3 ? "YES" : "NO", need_ur3 ? "YES" : "NO");

//     auto pending = std::make_shared<std::atomic<int>>(0);
//     if (need_rb3) pending->fetch_add(1);
//     if (need_ur3) pending->fetch_add(1);

//     auto after = [this, pending]() {
//       if (pending->fetch_sub(1) == 1) {
//         RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks done → FIRE BOTH step0", ts().c_str());
//         sync_inflight_ = std::make_pair(0u, 0u);
//         rb3_inflight_done_ = ur3_inflight_done_ = false;
//         fire_both_immediate(/*rb3_idx=*/0, /*ur3_idx=*/0);
//       }
//     };

//     if (need_rb3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [PREP] RB3 BEFORE hook (idx0)", ts().c_str());
//       request_rb_hand(rb3_grip_before_[0], after);
//     }
//     if (need_ur3) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [PREP] UR3e BEFORE hook (idx0)", ts().c_str());
//       request_ur_gripper(ur3_grip_before_[0], after);
//     }
//   }

//   // 0ms 타이머 대신 항상 즉시 발사로 통일하려면 이 함수는 안 써도 됩니다.
//   void fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks)
//   {
//     if (!apply_before_hooks) {
//       // 즉시 발사로 대체
//       fire_both_immediate(rb3_idx, ur3_idx);
//       return;
//     }

//     size_t pending = 0;
//     bool do_rb3_before = rb3_grip_before_.count(rb3_idx) &&
//                         rb3_grip_before_[rb3_idx] != GripperAction::NONE;
//     bool do_ur3_before = ur3_grip_before_.count(ur3_idx) &&
//                         ur3_grip_before_[ur3_idx] != GripperAction::NONE;

//     auto after_one_done = [&]() {
//       if (--pending == 0) {
//         fire_both_immediate(rb3_idx, ur3_idx);
//       }
//     };

//     if (do_rb3_before) pending++;
//     if (do_ur3_before) pending++;
//     if (pending == 0) {
//       fire_both_immediate(rb3_idx, ur3_idx);
//       return;
//     }
//     if (do_rb3_before) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] RB3 BEFORE hook (idx%zu)", ts().c_str(), rb3_idx);
//       request_rb_hand(rb3_grip_before_[rb3_idx], [after_one_done](){});
//     }
//     if (do_ur3_before) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] UR3e BEFORE hook (idx%zu)", ts().c_str(), ur3_idx);
//       request_ur_gripper(ur3_grip_before_[ur3_idx], [after_one_done](){});
//     }
//   }

//   // 타이머 없이 즉시 발사
//   void fire_both_immediate(size_t rb3_idx, size_t ur3_idx)
//   {
//     RCLCPP_INFO(this->get_logger(),
//       "[%s] FIRE BOTH IMMEDIATE (RB3=%zu, UR3e=%zu)",
//       ts().c_str(), rb3_idx, ur3_idx);

//     if (use_bimanual_srv_) {
//       const auto &r = seq_rb3_[rb3_idx];
//       const auto &l = seq_ur3_[ur3_idx];
//       call_zmk_move_to_bimanual_tcppos_async(
//         /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
//         /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
//     } else {
//       // 비바이매뉴얼 모드에서는 순서 오프셋이 생길 수 있으니
//       // 정말 ‘동시’가 중요하면 bimanual 모드를 권장
//       send_ur3_step(ur3_idx);
//       send_rb3_step(rb3_idx);
//     }
//   }

//   // ===== (동기) 양팔 동시 tcppos 호출 ====
//   void call_zmk_move_to_bimanual_tcppos(
//       double l_x, double l_y, double l_z, double l_rx, double l_ry, double l_rz,
//       double r_x, double r_y, double r_z, double r_rx, double r_ry, double r_rz)
//   {
//     auto req = std::make_shared<dual_arm_msg::srv::Movetcppos::Request>();
//     // 왼팔(UR3e)
//     req->left_x = l_x;  req->left_y = l_y;  req->left_z = l_z;
//     req->left_rx = l_rx; req->left_ry = l_ry; req->left_rz = l_rz;
//     // 오른팔(RB3)
//     req->right_x = r_x; req->right_y = r_y; req->right_z = r_z;
//     req->right_rx = r_rx; req->right_ry = r_ry; req->right_rz = r_rz;

//     if (!client_bimanual_tcppos_->wait_for_service(std::chrono::seconds(2))) {
//       RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] service not available: %s", bimanual_srv_name_.c_str());
//       return;
//     }

//     auto future = client_bimanual_tcppos_->async_send_request(req);
//     auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5));

//     if (ret != rclcpp::FutureReturnCode::SUCCESS || !future.valid()) {
//       RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] service timeout or invalid future");
//       return;
//     }

//     const auto resp = future.get();  // 반드시 1회만!
//     if (resp && resp->success) {
//       RCLCPP_INFO(this->get_logger(), "[BIMANUAL] move_tcppos success");
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] move_tcppos failed: %s",
//                    resp ? resp->message.c_str() : "no response");
//     }
//   }

//   // ===== (비동기) 상태머신 친화 래퍼 ====
//   void call_zmk_move_to_bimanual_tcppos_async(
//       double l_x, double l_y, double l_z, double l_rx, double l_ry, double l_rz,
//       double r_x, double r_y, double r_z, double r_rx, double r_ry, double r_rz)
//   {
//     auto req = std::make_shared<dual_arm_msg::srv::Movetcppos::Request>();
//     // 왼팔(UR3e)
//     req->left_x = l_x;  req->left_y = l_y;  req->left_z = l_z;
//     req->left_rx = l_rx; req->left_ry = l_ry; req->left_rz = l_rz;
//     // 오른팔(RB3)
//     req->right_x = r_x; req->right_y = r_y; req->right_z = r_z;
//     req->right_rx = r_rx; req->right_ry = r_ry; req->right_rz = r_rz;

//     // 상태머신 트리거
//     rb3_waiting_ = ur3_waiting_ = true;
//     rb3_seen_active_ = ur3_seen_active_ = false;
//     t_send_rb3_ = t_send_ur3_ = std::chrono::steady_clock::now();

//     RCLCPP_INFO(this->get_logger(),
//       "[%s] [BIMANUAL] SEND tcppos (UR3e & RB3) via %s", ts().c_str(), bimanual_srv_name_.c_str());

//     client_bimanual_tcppos_->async_send_request(
//       req,
//       [this](rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedFuture f){
//         bool ok = false; std::string msg = "no response";
//         try { auto resp = f.get(); ok = resp && resp->success; if (resp) msg = resp->message; }
//         catch (const std::exception& e) { msg = e.what(); }
//         catch (...) {}
//         RCLCPP_INFO(this->get_logger(), "[%s] [BIMANUAL] request SENT %s (%s)",
//                     ts().c_str(), ok ? "OK" : "FAILED", msg.c_str());
//       });
//   }

//   // ===== RB3 =====
//   void start_rb3_step(size_t i)
//   {
//     if (i >= seq_rb3_.size()) { if (!rb3_done_all_) { rb3_done_all_ = true; maybe_finish(); } return; }
//     if (rb3_grip_before_.count(i) && rb3_grip_before_[i] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB3] BEFORE hook at idx=%zu", ts().c_str(), i);
//       request_rb_hand(rb3_grip_before_[i], /*on_done=*/nullptr);
//       send_rb3_step(i);
//     } else {
//       send_rb3_step(i);
//     }
//   }
//   void send_rb3_step(size_t i)
//   {
//     if (i >= seq_rb3_.size()) return;
//     const auto &p = seq_rb3_[i];
//     auto req = std::make_shared<RB3Srv::Request>();
//     req->right_x = p.x; req->right_y = p.y; req->right_z = p.z;
//     req->right_rx = p.rx; req->right_ry = p.ry; req->right_rz = p.rz;

//     rb3_waiting_ = true;
//     rb3_seen_active_ = false;
//     t_send_rb3_ = std::chrono::steady_clock::now();

//     RCLCPP_INFO(this->get_logger(), "[%s] [RB3] SEND step %zu", ts().c_str(), i);
//     cli_rb3_->async_send_request(
//       req,
//       [this, i](rclcpp::Client<RB3Srv>::SharedFuture future) {
//         auto resp = future.get();
//         if (resp && resp->success) {
//           RCLCPP_INFO(this->get_logger(), "[%s] [RB3] step %zu SENT OK: %s",
//                       ts().c_str(), i, resp->message.c_str());
//         } else {
//           RCLCPP_ERROR(this->get_logger(), "[%s] [RB3] step %zu SEND FAILED", ts().c_str(), i);
//         }
//       });
//   }
//   void advance_rb3_after_done(size_t just_finished_idx)
//   {
//     if (rb3_grip_after_.count(just_finished_idx) &&
//         rb3_grip_after_[just_finished_idx] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB3] AFTER hook at idx=%zu", ts().c_str(), just_finished_idx);
//       request_rb_hand(rb3_grip_after_[just_finished_idx], [this]{ ++idx_rb3_; start_rb3_step(idx_rb3_); });
//     } else {
//       ++idx_rb3_;
//       start_rb3_step(idx_rb3_);
//     }
//   }

//   // ===== UR3e =====
//   void start_ur3_step(size_t i)
//   {
//     if (i >= seq_ur3_.size()) { if (!ur3_done_all_) { ur3_done_all_ = true; maybe_finish(); } return; }
//     if (ur3_grip_before_.count(i) && ur3_grip_before_[i] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] BEFORE hook at idx=%zu", ts().c_str(), i);
//       request_ur_gripper(ur3_grip_before_[i], [this, i]{ send_ur3_step(i); });
//     } else {
//       send_ur3_step(i);
//     }
//   }
//   void send_ur3_step(size_t i)
//   {
//     if (i >= seq_ur3_.size()) return;
//     const auto &p = seq_ur3_[i];
//     auto req = std::make_shared<UR3Srv::Request>();
//     req->left_x = p.x; req->left_y = p.y; req->left_z = p.z;
//     req->left_rx = p.rx; req->left_ry = p.ry; req->left_rz = p.rz;

//     ur3_waiting_ = true;
//     ur3_seen_active_ = false;
//     t_send_ur3_ = std::chrono::steady_clock::now();

//     RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] SEND step %zu", ts().c_str(), i);
//     cli_ur3_->async_send_request(
//       req,
//       [this, i](rclcpp::Client<UR3Srv>::SharedFuture future) {
//         auto resp = future.get();
//         if (resp && resp->success) {
//           RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] step %zu SENT OK: %s",
//                       ts().c_str(), i, resp->message.c_str());
//         } else {
//           RCLCPP_ERROR(this->get_logger(), "[%s] [UR3e] step %zu SEND FAILED", ts().c_str(), i);
//         }
//       });
//   }
//   void advance_ur3_after_done(size_t just_finished_idx)
//   {
//     if (ur3_grip_after_.count(just_finished_idx) &&
//         ur3_grip_after_[just_finished_idx] != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] AFTER hook at idx=%zu", ts().c_str(), just_finished_idx);
//       request_ur_gripper(ur3_grip_after_[just_finished_idx], [this]{ ++idx_ur3_; start_ur3_step(idx_ur3_); });
//     } else {
//       ++idx_ur3_;
//       start_ur3_step(idx_ur3_);
//     }
//   }

//   void run_after_hook_barrier_only(bool is_left, size_t finished_idx)
//   {
//     GripperAction act = GripperAction::NONE;
//     if (is_left) {
//       auto it = ur3_grip_after_.find(finished_idx);
//       if (it != ur3_grip_after_.end()) act = it->second;
//     } else {
//       auto it = rb3_grip_after_.find(finished_idx);
//       if (it != rb3_grip_after_.end()) act = it->second;
//     }
//     if (act != GripperAction::NONE) {
//       RCLCPP_INFO(this->get_logger(),
//         "[%s] [BARRIER AFTER] %s idx=%zu → %s",
//         ts().c_str(), is_left ? "UR3e" : "RB3", finished_idx,
//         act == GripperAction::OPEN ? "OPEN" : (act == GripperAction::CLOSE ? "CLOSE" : "PINCH"));
//       if (is_left) request_ur_gripper(act, /*on_done=*/nullptr);
//       else         request_rb_hand(act,    /*on_done=*/nullptr);
//     }
//   }

//   void maybe_finish() {
//     if (rb3_done_all_ && ur3_done_all_) {
//       RCLCPP_INFO(this->get_logger(), "[%s] === Sequence ALL DONE ===", ts().c_str());

//       if (exit_when_done_ && !shutdown_scheduled_) {
//         shutdown_scheduled_ = true;
//         shutdown_timer_ = this->create_wall_timer(
//           std::chrono::milliseconds(200),
//           [this]() {
//             RCLCPP_INFO(this->get_logger(), "[%s] Shutting down (exit_when_done=true).", ts().c_str());
//             rclcpp::shutdown();
//           });
//       }
//     }
//   }

//   // 상태 콜백
//   void on_right_status(GoalStatusArray::SharedPtr msg)
//   {
//     const bool active = any_active(*msg);
//     if (rb3_waiting_) {
//       if (active && !rb3_seen_active_) {
//         rb3_seen_active_ = true;
//         t_active_rb3_ = std::chrono::steady_clock::now();
//         auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_rb3_ - t_send_rb3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB3] FIRST ACTIVE (Δsend→active=%lldms)", ts().c_str(), (long long)d);
//       }
//       if (rb3_seen_active_ && !active) {
//         rb3_waiting_ = false;
//         auto now = std::chrono::steady_clock::now();
//         auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_rb3_).count();
//         auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_rb3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB3] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
//                     ts().c_str(), idx_rb3_, (long long)d1, (long long)d2);

//         if (sync_inflight_.has_value() && idx_rb3_ == sync_inflight_->first) {
//           run_after_hook_barrier_only(/*is_left=*/false, idx_rb3_);  // inflight after
//           rb3_inflight_done_ = true;
//           check_inflight_and_advance_after_both_done();
//           return;
//         }
//         if (handle_sync_reached(/*is_left=*/false, idx_rb3_)) return;

//         advance_rb3_after_done(idx_rb3_);
//       }
//     }
//   }
//   void on_left_status(GoalStatusArray::SharedPtr msg)
//   {
//     const bool active = any_active(*msg);
//     if (ur3_waiting_) {
//       if (active && !ur3_seen_active_) {
//         ur3_seen_active_ = true;
//         t_active_ur3_ = std::chrono::steady_clock::now();
//         auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_ur3_ - t_send_ur3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] FIRST ACTIVE (Δsend→active=%lldms)", ts().c_str(), (long long)d);
//       }
//       if (ur3_seen_active_ && !active) {
//         ur3_waiting_ = false;
//         auto now = std::chrono::steady_clock::now();
//         auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_ur3_).count();
//         auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_ur3_).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
//                     ts().c_str(), idx_ur3_, (long long)d1, (long long)d2);

//         if (sync_inflight_.has_value() && idx_ur3_ == sync_inflight_->second) {
//           run_after_hook_barrier_only(/*is_left=*/true, idx_ur3_);   // inflight after
//           ur3_inflight_done_ = true;
//           check_inflight_and_advance_after_both_done();
//           return;
//         }
//         if (handle_sync_reached(/*is_left=*/true, idx_ur3_)) return;

//         advance_ur3_after_done(idx_ur3_);
//       }
//     }
//   }

//   bool handle_sync_reached(bool is_left, size_t just_finished_idx)
//   {
//     bool matched_any = false;

//     for (auto &sp : sync_points_) {
//       if (sp.fired) continue;

//       if (!is_left && just_finished_idx == sp.rb3_idx) {
//         sp.reached_rb3 = true; matched_any = true;
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] RB3 reached idx=%zu", ts().c_str(), sp.rb3_idx);
//         run_after_hook_barrier_only(/*is_left=*/false, just_finished_idx);
//       }
//       if ( is_left && just_finished_idx == sp.ur3_idx) {
//         sp.reached_ur3 = true; matched_any = true;
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] UR3e reached idx=%zu", ts().c_str(), sp.ur3_idx);
//         run_after_hook_barrier_only(/*is_left=*/true, just_finished_idx);
//       }

//       if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
//         sp.fired = true;

//         idx_rb3_ = sp.rb3_next;
//         idx_ur3_ = sp.ur3_next;

//         sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//         rb3_inflight_done_ = false;
//         ur3_inflight_done_ = false;

//         RCLCPP_INFO(this->get_logger(),
//           "[%s] [SYNC] BOTH reached → FIRE NOW (RB3=%zu, UR3e=%zu)",
//           ts().c_str(), idx_rb3_, idx_ur3_);

//         // 즉시 동시 발사
//         fire_both_immediate(idx_rb3_, idx_ur3_);
//         return true;
//       }
//     }
//     return matched_any;
//   }

//   // 방금 끝난 인플라이트 쌍이 sync_points에 등록돼 있으면 동시 다음 스텝 발사
//   bool try_fire_next_sync_after_inflight_pair(size_t just_rb3_idx, size_t just_ur3_idx)
//   {
//     for (auto &sp : sync_points_) {
//       if (sp.rb3_idx == just_rb3_idx && sp.ur3_idx == just_ur3_idx) {
//         if (sp.fired) return false;   // 이미 처리됨
//         sp.fired = true;

//         idx_rb3_ = sp.rb3_next;
//         idx_ur3_ = sp.ur3_next;

//         sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//         rb3_inflight_done_ = ur3_inflight_done_ = false;

//         RCLCPP_INFO(this->get_logger(),
//           "[%s] [SYNC/POST-INFLIGHT] (RB3=%zu, UR3=%zu) → FIRE NEXT (RB3=%zu, UR3=%zu)",
//           ts().c_str(), just_rb3_idx, just_ur3_idx, idx_rb3_, idx_ur3_);

//         // 즉시 동시 발사
//         fire_both_immediate(idx_rb3_, idx_ur3_);
//         return true;
//       }
//     }
//     return false;
//   }

//   void check_inflight_and_advance_after_both_done()
//   {
//     if (!sync_inflight_.has_value()) return;
//     if (!(rb3_inflight_done_ && ur3_inflight_done_)) return;

//     auto [rb3_idx, ur3_idx] = *sync_inflight_;
//     RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] inflight done (RB3=%zu, UR3e=%zu)", ts().c_str(), rb3_idx, ur3_idx);

//     sync_inflight_.reset();
//     rb3_inflight_done_ = ur3_inflight_done_ = false;

//     // (RB3=4, UR3=2) 완료 시: 외부 서비스 콜 올 때까지 대기
//     if (rb3_idx == 4 && ur3_idx == 2) {
//       hold_until_degcheck_ = true;
//       RCLCPP_INFO(this->get_logger(), "[%s] [DEGCHECK] Holding at (RB3=4, UR3=2). "
//                                        "Call `/zmk_DegcheckFlag` to proceed.", ts().c_str());
//       return;
//     }

//     // 일반 규칙: 인플라이트로 끝난 쌍이 sync_points 엔트리라면 다음은 '동시 발사'
//     if (try_fire_next_sync_after_inflight_pair(rb3_idx, ur3_idx)) {
//       return;
//     }

//     // 그 외에는 개별 진행
//     ++idx_rb3_;  start_rb3_step(idx_rb3_);
//     ++idx_ur3_;  start_ur3_step(idx_ur3_);
//   }

//   // === UR(왼팔) 그리퍼(Trigger) ===
//   void request_ur_gripper(GripperAction action, std::function<void()> on_done)
//   {
//     if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
//     auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
//     auto cli = (action == GripperAction::CLOSE) ? cli_grip_close_ : cli_grip_open_;
//     if (!cli->wait_for_service(0s)) {
//       RCLCPP_WARN(this->get_logger(), "[%s] UR gripper service not available (%s).",
//                   ts().c_str(), action == GripperAction::CLOSE ? "CLOSE" : "OPEN");
//       if (on_done) on_done();
//       return;
//     }
//     auto t0 = std::chrono::steady_clock::now();
//     cli->async_send_request(
//       req,
//       [this, action, on_done, t0](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
//         bool ok = false;
//         try { auto resp = future.get(); ok = resp && resp->success; } catch (...) {}
//         auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
//                     std::chrono::steady_clock::now() - t0).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR %s] %s (Δcall=%lldms)",
//           ts().c_str(), (action == GripperAction::CLOSE ? "CLOSE" : "OPEN"), ok ? "OK" : "FAILED", (long long)dt);
//         if (on_done) on_done();
//       });
//   }

//   // === RB(오른팔) Inspire Hand (/Setangle) ===
//   void request_rb_hand(GripperAction action, std::function<void()> on_done)
//   {
//     if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
//     if (!cli_hand_setangle_->wait_for_service(0s)) {
//       RCLCPP_WARN(this->get_logger(), "[%s] RB hand service not available: %s",
//                   ts().c_str(), hand_service_name_.c_str());
//       if (on_done) on_done();
//       return;
//     }

//     auto req = std::make_shared<HandSetAngleSrv::Request>();
//     // 고정 프리셋
//     if (action == GripperAction::CLOSE)
//     {
//       req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 645;
//       req->angle3 = 671;  req->angle4 = 260;  req->angle5 = 0;
//     }
//     else if(action == GripperAction::PINCH)
//     {
//       req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
//       req->angle3 = 1000; req->angle4 = 400;  req->angle5 = 0;
//     }
//     else
//     { // OPEN
//       req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
//       req->angle3 = 1000; req->angle4 = 0; req->angle5 = 1000;
//     }
//     req->hand_id = 1;
//     req->status  = "set_angle";

//     const char* tag =
//        (action == GripperAction::CLOSE ? "CLOSE" :
//        action == GripperAction::OPEN  ? "OPEN"  : "PINCH");

//     auto t0 = std::chrono::steady_clock::now();
//     cli_hand_setangle_->async_send_request(
//       req,
//       [this, on_done, t0, tag](rclcpp::Client<HandSetAngleSrv>::SharedFuture future) {
//         bool ok = false;
//         try {
//           auto resp = future.get();
//           ok = (resp != nullptr);
//         } catch (...) {}
//         auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
//                     std::chrono::steady_clock::now() - t0).count();
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB HAND %s] %s (Δcall=%lldms)",
//           ts().c_str(), tag, ok ? "CALLED" : "FAILED", (long long)dt);
//         if (on_done) on_done();
//       }
//     );
//   }
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MoveSequenceClient>());
//   rclcpp::shutdown();
//   return 0;
// }






//--------------일단은 멈추고 재시작함(결합시 동시 발사)-------------------------

// #include <memory>
// #include <vector>
// #include <string>
// #include <chrono>
// #include <unordered_map>
// #include <functional>
// #include <optional>
// #include <atomic>
// #include <sstream>
// #include <iomanip>
// #include <thread>
// #include <future>

// #include <rclcpp/rclcpp.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <action_msgs/msg/goal_status_array.hpp>
// #include <action_msgs/srv/cancel_goal.hpp>

// #include "dual_arm_msg/srv/movetcppos_rb3.hpp"
// #include "dual_arm_msg/srv/movetcppos_ur3.hpp"
// #include "dual_arm_msg/srv/movetcppos.hpp"
// #include "inspire_hand_interface/srv/setangle.hpp"

// using namespace std::chrono_literals;
// using action_msgs::msg::GoalStatusArray;
// using action_msgs::msg::GoalStatus;

// using RB3Srv = dual_arm_msg::srv::MovetcpposRB3;
// using UR3Srv = dual_arm_msg::srv::MovetcpposUR3;
// using HandSetAngleSrv = inspire_hand_interface::srv::Setangle;

// struct Pose6 { double x, y, z, rx, ry, rz; };
// enum class GripperAction { NONE = 0, OPEN, CLOSE, PINCH, HOME};

// class MoveSequenceClient : public rclcpp::Node
// {
// public:
//   MoveSequenceClient();

// private:
//   // ---------- types ----------
//   struct PairSync {
//     size_t rb3_idx;
//     size_t ur3_idx;
//     size_t rb3_next;
//     size_t ur3_next;
//     bool reached_rb3 = false;
//     bool reached_ur3 = false;
//     bool fired = false;
//   };

//   // ---------- data ----------
//   std::vector<PairSync> sync_points_;
//   std::optional<std::pair<size_t,size_t>> sync_inflight_;
//   bool rb3_inflight_done_ = false;
//   bool ur3_inflight_done_ = false;

//   rclcpp::Client<RB3Srv>::SharedPtr cli_rb3_;
//   rclcpp::Client<UR3Srv>::SharedPtr cli_ur3_;
//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_open_, cli_grip_close_;
//   rclcpp::Client<HandSetAngleSrv>::SharedPtr cli_hand_setangle_;
//   std::string hand_service_name_;

//   rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedPtr client_bimanual_tcppos_;
//   std::string bimanual_srv_name_;
//   bool use_bimanual_srv_ = true;

//   rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_pickblock_;
//   rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_degcheck_;

//   rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_right_cli_, cancel_left_cli_;

//   rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_right_status_;
//   rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_left_status_;
//   rclcpp::TimerBase::SharedPtr start_timer_;
//   std::string right_fjt_base_, left_fjt_base_;
//   std::vector<Pose6> seq_rb3_, seq_ur3_;
//   size_t idx_rb3_ = 0, idx_ur3_ = 0;
//   bool rb3_waiting_ = false, ur3_waiting_ = false;
//   bool rb3_seen_active_ = false, ur3_seen_active_ = false;
//   bool rb3_done_all_ = false,  ur3_done_all_ = false;

//   std::unordered_map<size_t, GripperAction> ur3_grip_before_;
//   std::unordered_map<size_t, GripperAction> ur3_grip_after_;
//   std::unordered_map<size_t, GripperAction> rb3_grip_before_;
//   std::unordered_map<size_t, GripperAction> rb3_grip_after_;

//   rclcpp::TimerBase::SharedPtr sync_fire_timer_;

//   bool hold_until_degcheck_{false};

//   std::chrono::steady_clock::time_point t0_steady_;
//   std::chrono::steady_clock::time_point t_send_rb3_, t_send_ur3_;
//   std::chrono::steady_clock::time_point t_active_rb3_, t_active_ur3_;

//   bool exit_when_done_{true};
//   bool shutdown_scheduled_{false};
//   rclcpp::TimerBase::SharedPtr shutdown_timer_;

//   // ---- soft-restart flags ----
//   std::atomic<bool> restart_pending_{false};   // 재시작 요청이 들어옴
//   bool block_progress_{false};                 // 다음 step/발사 진행 금지
//   rclcpp::TimerBase::SharedPtr restart_watchdog_; // 재시작 워치독

//   // ---------- helpers (decl) ----------
//   std::string ts() const;
//   static bool ends_with(const std::string &s, const std::string &suffix);
//   static bool any_active(const GoalStatusArray &arr);
//   std::string resolve_status_topic(const std::string &base);

//   void start_sequence();
//   void reset_sequence_state();
//   bool cancel_all_goals();

//   void prepare_and_fire_first_pair();
//   void fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks);
//   void fire_both_immediate(size_t rb3_idx, size_t ur3_idx);

//   void call_zmk_move_to_bimanual_tcppos(double l_x,double l_y,double l_z,double l_rx,double l_ry,double l_rz,
//                                         double r_x,double r_y,double r_z,double r_rx,double r_ry,double r_rz);
//   void call_zmk_move_to_bimanual_tcppos_async(double l_x,double l_y,double l_z,double l_rx,double l_ry,double l_rz,
//                                               double r_x,double r_y,double r_z,double r_rx,double r_ry,double r_rz);

//   void start_rb3_step(size_t i);
//   void send_rb3_step(size_t i);
//   void advance_rb3_after_done(size_t just_finished_idx);

//   void start_ur3_step(size_t i);
//   void send_ur3_step(size_t i);
//   void advance_ur3_after_done(size_t just_finished_idx);

//   void run_after_hook_barrier_only(bool is_left, size_t finished_idx);
//   bool handle_sync_reached(bool is_left, size_t just_finished_idx);
//   bool try_fire_next_sync_after_inflight_pair(size_t just_rb3_idx, size_t just_ur3_idx);
//   void check_inflight_and_advance_after_both_done();
//   void maybe_finish();

//   void request_ur_gripper(GripperAction action, std::function<void()> on_done);
//   void request_rb_hand(GripperAction action, std::function<void()> on_done);

//   void on_right_status(GoalStatusArray::SharedPtr msg);
//   void on_left_status(GoalStatusArray::SharedPtr msg);

//   // soft-restart helpers
//   void request_soft_restart();     // 서비스에서 호출
//   void maybe_restart_after_idle(); // 양팔 idle 시 재시작
//   void do_restart_now();           // 실제 리셋 & 재시작
// };

// // ===================== IMPLEMENTATION =====================

// MoveSequenceClient::MoveSequenceClient()
// : Node("move_sequence_client_event_driven")
// , t0_steady_(std::chrono::steady_clock::now())
// {
//   exit_when_done_ = this->declare_parameter<bool>("exit_when_done", true);

//   right_fjt_base_ = declare_parameter<std::string>(
//     "right_fjt_base", "/joint_trajectory_controller/follow_joint_trajectory");
//   left_fjt_base_ = declare_parameter<std::string>(
//     "left_fjt_base", "/scaled_joint_trajectory_controller/follow_joint_trajectory");

//   cli_rb3_ = this->create_client<RB3Srv>("zmk_tcppos_rb3");
//   cli_ur3_ = this->create_client<UR3Srv>("zmk_tcppos_ur3e");

//   cli_grip_open_  = this->create_client<std_srvs::srv::Trigger>("zmk_girp_open");
//   cli_grip_close_ = this->create_client<std_srvs::srv::Trigger>("zmk_girp_close");

//   hand_service_name_ = declare_parameter<std::string>("hand_service_name", "/Setangle");
//   cli_hand_setangle_ = this->create_client<HandSetAngleSrv>(hand_service_name_);

//   bimanual_srv_name_ = declare_parameter<std::string>("bimanual_srv_name", "zmk_move_to_tcppos");
//   use_bimanual_srv_  = declare_parameter<bool>("use_bimanual_srv", true);
//   client_bimanual_tcppos_ = this->create_client<dual_arm_msg::srv::Movetcppos>(bimanual_srv_name_);

//   cancel_right_cli_ = this->create_client<action_msgs::srv::CancelGoal>(right_fjt_base_ + "/_action/cancel_goal");
//   cancel_left_cli_  = this->create_client<action_msgs::srv::CancelGoal>(left_fjt_base_  + "/_action/cancel_goal");

//   // ---- services ----
//   srv_degcheck_ = this->create_service<std_srvs::srv::Trigger>(
//     "zmk_DegcheckFlag",
//     [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
//            std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
//     {
//       if (!hold_until_degcheck_) {
//         resp->success = false;
//         resp->message = "Not holding at (RB3=4, UR3=2).";
//         return;
//       }
//       hold_until_degcheck_ = false;
//       RCLCPP_INFO(this->get_logger(), "[%s] [DEGCHECK] Release → FIRE (RB3=5, UR3=3)", this->ts().c_str());
//       idx_rb3_ = 5; idx_ur3_ = 3;
//       sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;
//       this->fire_both_immediate(idx_rb3_, idx_ur3_);
//       resp->success = true;
//       resp->message = "Proceed fired to (RB3=5, UR3=3).";
//     }
//   );
//   RCLCPP_INFO(get_logger(), "Service server ready: /zmk_DegcheckFlag");

//   srv_pickblock_ = this->create_service<std_srvs::srv::Trigger>(
//     "zmk_pickblockchkFlag",
//     [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
//            std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
//     {
//       RCLCPP_WARN(this->get_logger(), "[%s] [PICKBLOCK] Soft-restart requested: wait current motions → restart",
//                   this->ts().c_str());
//       this->request_soft_restart();
//       resp->success = true;
//       resp->message = "Soft-restart scheduled (will restart after current motions stop)";
//     }
//   );
//   RCLCPP_INFO(get_logger(), "Service server ready: /zmk_pickblockchkFlag");

//   // ---- sequences ----
//   seq_rb3_ = {
//     {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (0)
//     {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (1)
//     {-0.04642, -0.44152, 0.11987, 1.76208441,  0.24801129,  1.77238186}, // (2)
//     { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (3)
//     { 0.07944, -0.22198, 0.41277, 1.84917634, -1.4379768,  1.4158111},   // (4)
//     { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (5)
//     { 0.135,   -0.36851, 0.30087, 0.1446878,   0.1537635,   1.322436},   // (6)
//     { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (7)
//     {-0.04381, -0.45377, 0.13585, 1.75492856,  0.19338248,  1.8102555},  // (8)
//     {-0.0407,  -0.44568, 0.31162, 1.7228145,   0.17715092,  1.80205245}, // (9)
//     {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (10)
//   };
//   seq_ur3_ = {
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)
//     { -0.140,   -0.410,   0.175,   0.0,   -3.14,  0.0 },      // (1)
//     { -0.0800,  -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (2)
//     { -0.13056, -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (3)
//     { -0.185,   -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (4)
//     { -0.13056, -0.32030, 0.46642, 2.151,  0.109, -2.257 },   // (5)
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (6)
//   };

//   // gripper hooks
//   ur3_grip_before_[0] = GripperAction::OPEN;
//   ur3_grip_after_[1]  = GripperAction::CLOSE;
//   ur3_grip_after_[4]  = GripperAction::OPEN;

//   rb3_grip_after_[0]  = GripperAction::OPEN;
//   rb3_grip_before_[2] = GripperAction::PINCH;
//   rb3_grip_after_[2]  = GripperAction::CLOSE;
//   rb3_grip_before_[3] = GripperAction::CLOSE;
//   rb3_grip_after_[8]  = GripperAction::PINCH;
//   rb3_grip_after_[9]  = GripperAction::HOME;

//   // sync points
//   sync_points_.push_back(PairSync{/*rb3_idx=*/5, /*ur3_idx=*/3, /*rb3_next=*/6, /*ur3_next=*/4});
//   sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/2});

//   // bringup timer
//   start_timer_ = this->create_wall_timer(300ms, [this] {
//     if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s) ||
//         !cli_grip_open_->wait_for_service(0s) || !cli_grip_close_->wait_for_service(0s) ||
//         !cli_hand_setangle_->wait_for_service(0s) ||
//         (use_bimanual_srv_ && !client_bimanual_tcppos_->wait_for_service(0s))) {
//       RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//         "[%s] Waiting for services (rb3/ur3e/gripper/hand%s). hand_service=%s, bimanual=%s",
//         this->ts().c_str(), use_bimanual_srv_ ? "/bimanual" : "",
//         hand_service_name_.c_str(), bimanual_srv_name_.c_str());
//       return;
//     }
//     const auto right_status = this->resolve_status_topic(right_fjt_base_);
//     const auto left_status  = this->resolve_status_topic(left_fjt_base_);
//     if (right_status.empty() || left_status.empty()) {
//       RCLCPP_ERROR(this->get_logger(),
//         "[%s] Could not resolve action status topics. right_base=%s left_base=%s",
//         this->ts().c_str(), right_fjt_base_.c_str(), left_fjt_base_.c_str());
//       return;
//     }
//     RCLCPP_INFO(this->get_logger(), "[%s] [RESOLVED] RIGHT status: %s", this->ts().c_str(), right_status.c_str());
//     RCLCPP_INFO(this->get_logger(), "[%s] [RESOLVED] LEFT  status: %s", this->ts().c_str(), left_status.c_str());

//     sub_right_status_ = this->create_subscription<GoalStatusArray>(
//       right_status, rclcpp::QoS(50),
//       std::bind(&MoveSequenceClient::on_right_status, this, std::placeholders::_1));
//     sub_left_status_ = this->create_subscription<GoalStatusArray>(
//       left_status, rclcpp::QoS(50),
//       std::bind(&MoveSequenceClient::on_left_status, this, std::placeholders::_1));

//     RCLCPP_INFO(this->get_logger(), "[%s] Services ready. Using hand_service=%s, bimanual=%s (use=%s)",
//                 this->ts().c_str(), hand_service_name_.c_str(), bimanual_srv_name_.c_str(),
//                 use_bimanual_srv_ ? "true" : "false");

//     start_timer_->cancel();
//     this->start_sequence();
//   });
// }

// // ---- helpers ----
// std::string MoveSequenceClient::ts() const {
//   using namespace std::chrono;
//   auto now = steady_clock::now();
//   auto ms  = duration_cast<milliseconds>(now - t0_steady_).count();
//   std::ostringstream oss; oss << std::setw(9) << ms << "ms";
//   return oss.str();
// }
// bool MoveSequenceClient::ends_with(const std::string &s, const std::string &suffix) {
//   return s.size() >= suffix.size() &&
//          s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
// }
// bool MoveSequenceClient::any_active(const GoalStatusArray &arr) {
//   for (const auto &st : arr.status_list) {
//     if (st.status == GoalStatus::STATUS_ACCEPTED ||
//         st.status == GoalStatus::STATUS_EXECUTING ||
//         st.status == GoalStatus::STATUS_CANCELING) return true;
//   }
//   return false;
// }
// std::string MoveSequenceClient::resolve_status_topic(const std::string &base)
// {
//   const auto a = base + "/_action/status";
//   const auto b = base + "/status";
//   auto graph = this->get_topic_names_and_types();
//   if (graph.find(a) != graph.end()) return a;
//   if (graph.find(b) != graph.end()) return b;
//   for (const auto &kv : graph) {
//     const auto &topic = kv.first;
//     if ((ends_with(topic, "/_action/status") || ends_with(topic, "/status")) &&
//         topic.find(base) != std::string::npos) {
//       return topic;
//     }
//   }
//   return std::string();
// }

// // ---- restart/cancel ----
// void MoveSequenceClient::start_sequence() {
//   RCLCPP_INFO(this->get_logger(), "[%s] === Sequence PREP ===", this->ts().c_str());
//   // 재시작 시 플래그 초기화
//   block_progress_ = false;
//   this->prepare_and_fire_first_pair();
// }
// void MoveSequenceClient::reset_sequence_state() {
//   rb3_waiting_ = ur3_waiting_ = false;
//   rb3_seen_active_ = ur3_seen_active_ = false;
//   rb3_done_all_ = ur3_done_all_ = false;
//   idx_rb3_ = idx_ur3_ = 0;
//   hold_until_degcheck_ = false;
//   for (auto &sp : sync_points_) { sp.reached_rb3=false; sp.reached_ur3=false; sp.fired=false; }
//   sync_inflight_.reset();
//   if (sync_fire_timer_) sync_fire_timer_->cancel();
//   RCLCPP_INFO(this->get_logger(), "[%s] [RESET] sequence state cleared.", this->ts().c_str());
// }
// bool MoveSequenceClient::cancel_all_goals()
// {
//   if (!cancel_right_cli_->wait_for_service(500ms) ||
//       !cancel_left_cli_->wait_for_service(500ms)) {
//     RCLCPP_ERROR(this->get_logger(), "[%s] CancelGoal service not available", this->ts().c_str());
//     return false;
//   }
//   auto make_cancel_req = [](){
//     auto req = std::make_shared<action_msgs::srv::CancelGoal::Request>();
//     for (auto &b : req->goal_info.goal_id.uuid) b = 0;
//     req->goal_info.stamp.sec = 0;
//     req->goal_info.stamp.nanosec = 0;
//     return req;
//   };
//   auto f_r = cancel_right_cli_->async_send_request(make_cancel_req());
//   auto f_l = cancel_left_cli_->async_send_request(make_cancel_req());

//   bool ok_r = (f_r.wait_for(1s) == std::future_status::ready);
//   bool ok_l = (f_l.wait_for(1s) == std::future_status::ready);
//   if (!ok_r || !ok_l) {
//     RCLCPP_ERROR(this->get_logger(), "[%s] CancelGoal timeout (right=%s left=%s)",
//                  this->ts().c_str(), ok_r?"ok":"timeout", ok_l?"ok":"timeout");
//     return false;
//   }
//   try {
//     auto r = f_r.get(); auto l = f_l.get();
//     RCLCPP_INFO(this->get_logger(), "[%s] CancelGoal ret: right(code=%d) left(code=%d)",
//                 this->ts().c_str(), r->return_code, l->return_code);
//   } catch (...) {
//     RCLCPP_WARN(this->get_logger(), "[%s] CancelGoal exception (ignored)", this->ts().c_str());
//   }
//   std::this_thread::sleep_for(100ms);
//   return true;
// }

// // ---- soft-restart helpers ----
// void MoveSequenceClient::request_soft_restart()
// {
//   restart_pending_.store(true);
//   block_progress_ = true; // 이후 단계 진행 금지

//   // 워치독 시작: 둘 다 idle 되는 시점 감시
//   if (!restart_watchdog_ || restart_watchdog_->is_canceled()) {
//     restart_watchdog_ = this->create_wall_timer(
//       100ms, [this](){
//         if (!restart_pending_.load()) { restart_watchdog_->cancel(); return; }
//         if (!rb3_waiting_ && !ur3_waiting_) {
//           RCLCPP_INFO(this->get_logger(), "[%s] [RESTART WD] both idle → restart now", this->ts().c_str());
//           restart_watchdog_->cancel();
//           do_restart_now();
//         }
//       }
//     );
//   }

//   // 이미 idle이면 즉시 재시작
//   maybe_restart_after_idle();
// }

// void MoveSequenceClient::maybe_restart_after_idle()
// {
//   if (restart_pending_.load() && !rb3_waiting_ && !ur3_waiting_) {
//     do_restart_now();
//   }
// }

// void MoveSequenceClient::do_restart_now()
// {
//   if (!restart_pending_.load()) return;
//   RCLCPP_WARN(this->get_logger(), "[%s] [PICKBLOCK] Soft-restart NOW → reset & start from 0", this->ts().c_str());
//   this->reset_sequence_state();
//   restart_pending_.store(false);
//   block_progress_ = false;
//   this->start_sequence();
// }

// // ---- first pair ----
// void MoveSequenceClient::prepare_and_fire_first_pair() {
//   // 재시작 대기중이면 아무것도 발사하지 않음(양팔 idle 될 때까지 대기)
//   if (restart_pending_.load()) {
//     RCLCPP_INFO(this->get_logger(), "[%s] Soft-restart pending → not firing first pair", this->ts().c_str());
//     return;
//   }

//   const bool need_rb3 = rb3_grip_before_.count(0) && rb3_grip_before_[0] != GripperAction::NONE;
//   const bool need_ur3 = ur3_grip_before_.count(0) && ur3_grip_before_[0] != GripperAction::NONE;

//   if (!need_rb3 && !need_ur3) {
//     RCLCPP_INFO(this->get_logger(), "[%s] No BEFORE hooks at idx0 → FIRE BOTH step0 now", this->ts().c_str());
//     sync_inflight_ = std::make_pair(0u, 0u);
//     rb3_inflight_done_ = ur3_inflight_done_ = false;
//     this->fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
//     return;
//   }

//   RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks at idx0: RB3=%s, UR3=%s",
//     this->ts().c_str(), need_rb3 ? "YES" : "NO", need_ur3 ? "YES" : "NO");

//   auto pending = std::make_shared<std::atomic<int>>(0);
//   if (need_rb3) pending->fetch_add(1);
//   if (need_ur3) pending->fetch_add(1);

//   auto after = [this, pending]() {
//     if (pending->fetch_sub(1) == 1) {
//       RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks done → FIRE BOTH step0", this->ts().c_str());
//       sync_inflight_ = std::make_pair(0u, 0u);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;
//       this->fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
//     }
//   };

//   if (need_rb3) {
//     RCLCPP_INFO(this->get_logger(), "[%s] [PREP] RB3 BEFORE hook (idx0)", this->ts().c_str());
//     this->request_rb_hand(rb3_grip_before_[0], after);
//   }
//   if (need_ur3) {
//     RCLCPP_INFO(this->get_logger(), "[%s] [PREP] UR3e BEFORE hook (idx0)", this->ts().c_str());
//     this->request_ur_gripper(ur3_grip_before_[0], after);
//   }
// }

// // ---- concurrent fire ----
// void MoveSequenceClient::fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks)
// {
//   if (restart_pending_.load()) {
//     RCLCPP_INFO(this->get_logger(), "[%s] Soft-restart pending → skip FIRE BOTH", this->ts().c_str());
//     return;
//   }

//   if (!apply_before_hooks) {
//     sync_fire_timer_ = this->create_wall_timer(
//       0ms,
//       [this, rb3_idx, ur3_idx](){
//         if (restart_pending_.load()) { sync_fire_timer_->cancel(); return; }
//         RCLCPP_INFO(this->get_logger(),
//           "[%s] FIRE BOTH NOW (RB3=%zu, UR3e=%zu) — same tick",
//           this->ts().c_str(), rb3_idx, ur3_idx);

//         if (use_bimanual_srv_) {
//           const auto &r = seq_rb3_[rb3_idx];
//           const auto &l = seq_ur3_[ur3_idx];
//           this->call_zmk_move_to_bimanual_tcppos_async(
//             /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
//             /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
//         } else {
//           this->send_ur3_step(ur3_idx);
//           this->send_rb3_step(rb3_idx);
//         }
//         sync_fire_timer_->cancel();
//       }
//     );
//     return;
//   }

//   size_t pending = 0;
//   bool do_rb3_before = rb3_grip_before_.count(rb3_idx) && rb3_grip_before_[rb3_idx] != GripperAction::NONE;
//   bool do_ur3_before = ur3_grip_before_.count(ur3_idx) && ur3_grip_before_[ur3_idx] != GripperAction::NONE;

//   auto after_one_done = [this, &pending, rb3_idx, ur3_idx]() mutable {
//     if (--pending == 0) {
//       sync_fire_timer_ = this->create_wall_timer(
//         0ms,
//         [this, rb3_idx, ur3_idx](){
//           if (restart_pending_.load()) { sync_fire_timer_->cancel(); return; }
//           RCLCPP_INFO(this->get_logger(),
//             "[%s] FIRE BOTH (after hooks) (RB3=%zu, UR3e=%zu)", this->ts().c_str(), rb3_idx, ur3_idx);

//           if (use_bimanual_srv_) {
//             const auto &r = seq_rb3_[rb3_idx];
//             const auto &l = seq_ur3_[ur3_idx];
//             this->call_zmk_move_to_bimanual_tcppos_async(
//               /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
//               /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
//           } else {
//             this->send_rb3_step(rb3_idx);
//             this->send_ur3_step(ur3_idx);
//           }
//           sync_fire_timer_->cancel();
//         }
//       );
//     }
//   };

//   if (do_rb3_before) pending++;
//   if (do_ur3_before) pending++;
//   if (pending == 0) {
//     sync_fire_timer_ = this->create_wall_timer(
//       0ms, [this, rb3_idx, ur3_idx](){
//         if (restart_pending_.load()) { sync_fire_timer_->cancel(); return; }
//         RCLCPP_INFO(this->get_logger(),
//           "[%s] FIRE BOTH (no hooks) (RB3=%zu, UR3e=%zu)", this->ts().c_str(), rb3_idx, ur3_idx);
//         if (use_bimanual_srv_) {
//           const auto &r = seq_rb3_[rb3_idx];
//           const auto &l = seq_ur3_[ur3_idx];
//           this->call_zmk_move_to_bimanual_tcppos_async(
//             /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
//             /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
//         } else {
//           this->send_rb3_step(rb3_idx);
//           this->send_ur3_step(ur3_idx);
//         }
//         sync_fire_timer_->cancel();
//       }
//     );
//     return;
//   }
//   if (do_rb3_before) {
//     RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] RB3 BEFORE hook (idx%zu)", this->ts().c_str(), rb3_idx);
//     this->request_rb_hand(rb3_grip_before_[rb3_idx], [after_one_done](){});
//   }
//   if (do_ur3_before) {
//     RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] UR3e BEFORE hook (idx%zu)", this->ts().c_str(), ur3_idx);
//     this->request_ur_gripper(ur3_grip_before_[ur3_idx], [after_one_done](){});
//   }
// }

// void MoveSequenceClient::fire_both_immediate(size_t rb3_idx, size_t ur3_idx)
// {
//   if (restart_pending_.load()) {
//     RCLCPP_INFO(this->get_logger(), "[%s] Soft-restart pending → skip FIRE BOTH IMMEDIATE", this->ts().c_str());
//     return;
//   }

//   RCLCPP_INFO(this->get_logger(),
//     "[%s] FIRE BOTH IMMEDIATE (RB3=%zu, UR3e=%zu)", this->ts().c_str(), rb3_idx, ur3_idx);

//   if (use_bimanual_srv_) {
//     const auto &r = seq_rb3_[rb3_idx];
//     const auto &l = seq_ur3_[ur3_idx];
//     this->call_zmk_move_to_bimanual_tcppos_async(
//       /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
//       /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
//   } else {
//     this->send_ur3_step(ur3_idx);
//     this->send_rb3_step(rb3_idx);
//   }
// }

// // ---- bimanual calls ----
// void MoveSequenceClient::call_zmk_move_to_bimanual_tcppos(
//   double l_x,double l_y,double l_z,double l_rx,double l_ry,double l_rz,
//   double r_x,double r_y,double r_z,double r_rx,double r_ry,double r_rz)
// {
//   auto req = std::make_shared<dual_arm_msg::srv::Movetcppos::Request>();
//   req->left_x = l_x;  req->left_y = l_y;  req->left_z = l_z;
//   req->left_rx = l_rx; req->left_ry = l_ry; req->left_rz = l_rz;
//   req->right_x = r_x; req->right_y = r_y; req->right_z = r_z;
//   req->right_rx = r_rx; req->right_ry = r_ry; req->right_rz = r_rz;

//   if (!client_bimanual_tcppos_->wait_for_service(2s)) {
//     RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] service not available: %s", bimanual_srv_name_.c_str());
//     return;
//   }
//   auto future = client_bimanual_tcppos_->async_send_request(req);
//   auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s);
//   if (ret != rclcpp::FutureReturnCode::SUCCESS || !future.valid()) {
//     RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] service timeout/invalid");
//     return;
//   }
//   const auto resp = future.get();
//   if (resp && resp->success) {
//     RCLCPP_INFO(this->get_logger(), "[BIMANUAL] move_tcppos success");
//   } else {
//     RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] move_tcppos failed: %s",
//                  resp ? resp->message.c_str() : "no response");
//   }
// }

// void MoveSequenceClient::call_zmk_move_to_bimanual_tcppos_async(
//   double l_x,double l_y,double l_z,double l_rx,double l_ry,double l_rz,
//   double r_x,double r_y,double r_z,double r_rx,double r_ry,double r_rz)
// {
//   auto req = std::make_shared<dual_arm_msg::srv::Movetcppos::Request>();
//   req->left_x = l_x;  req->left_y = l_y;  req->left_z = l_z;
//   req->left_rx = l_rx; req->left_ry = l_ry; req->left_rz = l_rz;
//   req->right_x = r_x; req->right_y = r_y; req->right_z = r_z;
//   req->right_rx = r_rx; req->right_ry = r_ry; req->right_rz = r_rz;

//   rb3_waiting_ = ur3_waiting_ = true;
//   rb3_seen_active_ = ur3_seen_active_ = false;
//   t_send_rb3_ = t_send_ur3_ = std::chrono::steady_clock::now();

//   RCLCPP_INFO(this->get_logger(),
//     "[%s] [BIMANUAL] SEND tcppos (UR3e & RB3) via %s", this->ts().c_str(), bimanual_srv_name_.c_str());

//   client_bimanual_tcppos_->async_send_request(
//     req,
//     [this](rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedFuture f){
//       bool ok = false; std::string msg = "no response";
//       try { auto resp = f.get(); ok = resp && resp->success; if (resp) msg = resp->message; }
//       catch (const std::exception& e) { msg = e.what(); }
//       catch (...) {}
//       RCLCPP_INFO(this->get_logger(), "[%s] [BIMANUAL] request SENT %s (%s)",
//                   this->ts().c_str(), ok ? "OK" : "FAILED", msg.c_str());
//     });
// }

// // ---- RB3 ----
// void MoveSequenceClient::start_rb3_step(size_t i)
// {
//   if (i >= seq_rb3_.size()) { if (!rb3_done_all_) { rb3_done_all_ = true; this->maybe_finish(); } return; }
//   if (rb3_grip_before_.count(i) && rb3_grip_before_[i] != GripperAction::NONE) {
//     RCLCPP_INFO(this->get_logger(), "[%s] [RB3] BEFORE hook at idx=%zu", this->ts().c_str(), i);
//     this->request_rb_hand(rb3_grip_before_[i], /*on_done=*/nullptr);
//     this->send_rb3_step(i);
//   } else {
//     this->send_rb3_step(i);
//   }
// }
// void MoveSequenceClient::send_rb3_step(size_t i)
// {
//   if (i >= seq_rb3_.size()) return;
//   const auto &p = seq_rb3_[i];
//   auto req = std::make_shared<RB3Srv::Request>();
//   req->right_x = p.x; req->right_y = p.y; req->right_z = p.z;
//   req->right_rx = p.rx; req->right_ry = p.ry; req->right_rz = p.rz;

//   rb3_waiting_ = true;
//   rb3_seen_active_ = false;
//   t_send_rb3_ = std::chrono::steady_clock::now();

//   RCLCPP_INFO(this->get_logger(), "[%s] [RB3] SEND step %zu", this->ts().c_str(), i);
//   cli_rb3_->async_send_request(
//     req,
//     [this, i](rclcpp::Client<RB3Srv>::SharedFuture future) {
//       auto resp = future.get();
//       if (resp && resp->success) {
//         RCLCPP_INFO(this->get_logger(), "[%s] [RB3] step %zu SENT OK: %s",
//                     this->ts().c_str(), i, resp->message.c_str());
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "[%s] [RB3] step %zu SEND FAILED", this->ts().c_str(), i);
//       }
//     });
// }
// void MoveSequenceClient::advance_rb3_after_done(size_t just_finished_idx)
// {
//   // 소프트 재시작이면 진행 멈춤(그립퍼 훅도 스킵)
//   if (restart_pending_.load() || block_progress_) {
//     rb3_waiting_ = false;
//     maybe_restart_after_idle();
//     return;
//   }
//   if (rb3_grip_after_.count(just_finished_idx) &&
//       rb3_grip_after_[just_finished_idx] != GripperAction::NONE) {
//     RCLCPP_INFO(this->get_logger(), "[%s] [RB3] AFTER hook at idx=%zu", this->ts().c_str(), just_finished_idx);
//     this->request_rb_hand(rb3_grip_after_[just_finished_idx], [this]{ ++idx_rb3_; this->start_rb3_step(idx_rb3_); });
//   } else {
//     ++idx_rb3_;
//     this->start_rb3_step(idx_rb3_);
//   }
// }

// // ---- UR3e ----
// void MoveSequenceClient::start_ur3_step(size_t i)
// {
//   if (i >= seq_ur3_.size()) { if (!ur3_done_all_) { ur3_done_all_ = true; this->maybe_finish(); } return; }
//   if (ur3_grip_before_.count(i) && ur3_grip_before_[i] != GripperAction::NONE) {
//     RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] BEFORE hook at idx=%zu", this->ts().c_str(), i);
//     this->request_ur_gripper(ur3_grip_before_[i], [this, i]{ this->send_ur3_step(i); });
//   } else {
//     this->send_ur3_step(i);
//   }
// }
// void MoveSequenceClient::send_ur3_step(size_t i)
// {
//   if (i >= seq_ur3_.size()) return;
//   const auto &p = seq_ur3_[i];
//   auto req = std::make_shared<UR3Srv::Request>();
//   req->left_x = p.x; req->left_y = p.y; req->left_z = p.z;
//   req->left_rx = p.rx; req->left_ry = p.ry; req->left_rz = p.rz;

//   ur3_waiting_ = true;
//   ur3_seen_active_ = false;
//   t_send_ur3_ = std::chrono::steady_clock::now();

//   RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] SEND step %zu", this->ts().c_str(), i);
//   cli_ur3_->async_send_request(
//     req,
//     [this, i](rclcpp::Client<UR3Srv>::SharedFuture future) {
//       auto resp = future.get();
//       if (resp && resp->success) {
//         RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] step %zu SENT OK: %s",
//                     this->ts().c_str(), i, resp->message.c_str());
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "[%s] [UR3e] step %zu SEND FAILED", this->ts().c_str(), i);
//       }
//     });
// }
// void MoveSequenceClient::advance_ur3_after_done(size_t just_finished_idx)
// {
//   // 소프트 재시작이면 진행 멈춤(그립퍼 훅도 스킵)
//   if (restart_pending_.load() || block_progress_) {
//     ur3_waiting_ = false;
//     maybe_restart_after_idle();
//     return;
//   }
//   if (ur3_grip_after_.count(just_finished_idx) &&
//       ur3_grip_after_[just_finished_idx] != GripperAction::NONE) {
//     RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] AFTER hook at idx=%zu", this->ts().c_str(), just_finished_idx);
//     this->request_ur_gripper(ur3_grip_after_[just_finished_idx], [this]{ ++idx_ur3_; this->start_ur3_step(idx_ur3_); });
//   } else {
//     ++idx_ur3_;
//     this->start_ur3_step(idx_ur3_);
//   }
// }

// // ---- sync / inflight ----
// void MoveSequenceClient::run_after_hook_barrier_only(bool is_left, size_t finished_idx)
// {
//   GripperAction act = GripperAction::NONE;
//   if (is_left) {
//     auto it = ur3_grip_after_.find(finished_idx);
//     if (it != ur3_grip_after_.end()) act = it->second;
//   } else {
//     auto it = rb3_grip_after_.find(finished_idx);
//     if (it != rb3_grip_after_.end()) act = it->second;
//   }
//   if (act != GripperAction::NONE) {
//     RCLCPP_INFO(this->get_logger(),
//       "[%s] [BARRIER AFTER] %s idx=%zu → %s",
//       this->ts().c_str(), is_left ? "UR3e" : "RB3", finished_idx,
//       act == GripperAction::OPEN ? "OPEN" : (act == GripperAction::CLOSE ? "CLOSE" : "PINCH"));
//     if (is_left) this->request_ur_gripper(act, /*on_done=*/nullptr);
//     else         this->request_rb_hand(act,    /*on_done=*/nullptr);
//   }
// }

// bool MoveSequenceClient::handle_sync_reached(bool is_left, size_t just_finished_idx)
// {
//   bool matched_any = false;
//   for (auto &sp : sync_points_) {
//     if (sp.fired) continue;

//     if (!is_left && just_finished_idx == sp.rb3_idx) {
//       sp.reached_rb3 = true; matched_any = true;
//       RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] RB3 reached idx=%zu", this->ts().c_str(), sp.rb3_idx);
//       this->run_after_hook_barrier_only(/*is_left=*/false, just_finished_idx);
//     }
//     if ( is_left && just_finished_idx == sp.ur3_idx) {
//       sp.reached_ur3 = true; matched_any = true;
//       RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] UR3e reached idx=%zu", this->ts().c_str(), sp.ur3_idx);
//       this->run_after_hook_barrier_only(/*is_left=*/true, just_finished_idx);
//     }

//     if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
//       sp.fired = true;
//       // 재시작 대기면 여기서 다음 발사 금지
//       if (restart_pending_.load() || block_progress_) {
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] Soft-restart pending → not firing next pair", this->ts().c_str());
//         return true;
//       }
//       idx_rb3_ = sp.rb3_next;
//       idx_ur3_ = sp.ur3_next;

//       sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;

//       RCLCPP_INFO(this->get_logger(),
//         "[%s] [SYNC] BOTH reached → FIRE NOW (RB3=%zu, UR3e=%zu)",
//         this->ts().c_str(), idx_rb3_, idx_ur3_);

//       this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
//       return true;
//     }
//   }
//   return matched_any;
// }

// // 클래스 private 메서드에 추가
// bool MoveSequenceClient::try_fire_next_sync_after_inflight_pair(size_t just_rb3_idx, size_t just_ur3_idx)
// {
//   for (auto &sp : sync_points_) {
//     if (sp.rb3_idx == just_rb3_idx && sp.ur3_idx == just_ur3_idx) {
//       if (sp.fired) return false;   // 이미 처리됨
//       sp.fired = true;

//       // 재시작 대기면 여기서 멈춤
//       if (restart_pending_.load() || block_progress_) {
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] Soft-restart pending → not firing next pair", this->ts().c_str());
//         return true;
//       }

//       idx_rb3_ = sp.rb3_next;
//       idx_ur3_ = sp.ur3_next;

//       sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;

//       RCLCPP_INFO(this->get_logger(),
//         "[%s] [SYNC/POST-INFLIGHT] (RB3=%zu, UR3=%zu) → FIRE NEXT (RB3=%zu, UR3=%zu)",
//         this->ts().c_str(), just_rb3_idx, just_ur3_idx, idx_rb3_, idx_ur3_);

//       // 동시 발사
//       this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
//       return true;
//     }
//   }
//   return false;
// }

// void MoveSequenceClient::check_inflight_and_advance_after_both_done()
// {
//   if (!sync_inflight_.has_value()) return;
//   if (!(rb3_inflight_done_ && ur3_inflight_done_)) return;

//   auto [rb3_idx, ur3_idx] = *sync_inflight_;
//   RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] inflight done (RB3=%zu, UR3e=%zu)", this->ts().c_str(), rb3_idx, ur3_idx);

//   // (RB3=4, UR3=2) 특수 홀드
//   if (rb3_idx == 4 && ur3_idx == 2) {
//     hold_until_degcheck_ = true;
//     sync_inflight_.reset();
//     RCLCPP_INFO(this->get_logger(), "[%s] [DEGCHECK] Holding at (RB3=4, UR3=2). Call `/zmk_DegcheckFlag`.", this->ts().c_str());
//     return;
//   }

//   // ★★★ 여기서 '해당 인플라이트 쌍'이 sync_points_ 엔트리면 다음 쌍을 동시 발사
//   if (try_fire_next_sync_after_inflight_pair(rb3_idx, ur3_idx)) {
//     return; // 이미 다음 쌍 동시발사 했으므로 끝
//   }

//   // 이하 기존 로직
//   sync_inflight_.reset();
//   rb3_inflight_done_ = ur3_inflight_done_ = false;

//   if (restart_pending_.load() || block_progress_) {
//     RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] Soft-restart pending → hold after inflight", this->ts().c_str());
//     rb3_waiting_ = false;
//     ur3_waiting_ = false;
//     maybe_restart_after_idle();
//     return;
//   }

//   ++idx_rb3_;  this->start_rb3_step(idx_rb3_);
//   ++idx_ur3_;  this->start_ur3_step(idx_ur3_);
// }


// void MoveSequenceClient::maybe_finish() {
//   if (rb3_done_all_ && ur3_done_all_) {
//     RCLCPP_INFO(this->get_logger(), "[%s] === Sequence ALL DONE ===", this->ts().c_str());
//     if (exit_when_done_ && !shutdown_scheduled_) {
//       shutdown_scheduled_ = true;
//       shutdown_timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(200),
//         [this]() {
//           RCLCPP_INFO(this->get_logger(), "[%s] Shutting down (exit_when_done=true).", this->ts().c_str());
//           rclcpp::shutdown();
//         });
//     }
//   }
// }

// // ---- services to grippers ----
// void MoveSequenceClient::request_ur_gripper(GripperAction action, std::function<void()> on_done)
// {
//   if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
//   auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
//   auto cli = (action == GripperAction::CLOSE) ? cli_grip_close_ : cli_grip_open_;
//   if (!cli->wait_for_service(0s)) {
//     RCLCPP_WARN(this->get_logger(), "[%s] UR gripper service not available (%s).",
//                 this->ts().c_str(), action == GripperAction::CLOSE ? "CLOSE" : "OPEN");
//     if (on_done) on_done();
//     return;
//   }
//   auto t0 = std::chrono::steady_clock::now();
//   cli->async_send_request(
//     req,
//     [this, action, on_done, t0](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
//       bool ok = false;
//       try { auto resp = future.get(); ok = resp && resp->success; } catch (...) {}
//       auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
//                   std::chrono::steady_clock::now() - t0).count();
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR %s] %s (Δcall=%lldms)",
//         this->ts().c_str(), (action == GripperAction::CLOSE ? "CLOSE" : "OPEN"), ok ? "OK" : "FAILED", (long long)dt);
//       if (on_done) on_done();
//     });
// }

// void MoveSequenceClient::request_rb_hand(GripperAction action, std::function<void()> on_done)
// {
//   if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
//   if (!cli_hand_setangle_->wait_for_service(0s)) {
//     RCLCPP_WARN(this->get_logger(), "[%s] RB hand service not available: %s",
//                 this->ts().c_str(), hand_service_name_.c_str());
//     if (on_done) on_done();
//     return;
//   }

//   auto req = std::make_shared<HandSetAngleSrv::Request>();
//   if (action == GripperAction::CLOSE)
//   {
//     req->angle0 = 0; req->angle1 = 0; req->angle2 = 645;
//     req->angle3 = 671;  req->angle4 = 260;  req->angle5 = 0;
//   }
//   else if(action == GripperAction::PINCH)
//   {
//     req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
//     req->angle3 = 1000; req->angle4 = 400;  req->angle5 = 0;
//   }
//   else if(action == GripperAction::HOME)
//   {
//     req->angle0 = 0; req->angle1 = 0; req->angle2 = 0;
//     req->angle3 = 0; req->angle4 = 0;  req->angle5 = 1000;
//   }
//   else
//   { // OPEN
//     req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
//     req->angle3 = 1000; req->angle4 = 0; req->angle5 = 1000;
//   }
//   req->hand_id = 1;
//   req->status  = "set_angle";

//   const char* tag = (action == GripperAction::CLOSE ? "CLOSE" :
//                     action == GripperAction::OPEN  ? "OPEN"  : "PINCH");

//   auto t0 = std::chrono::steady_clock::now();
//   cli_hand_setangle_->async_send_request(
//     req,
//     [this, on_done, t0, tag](rclcpp::Client<HandSetAngleSrv>::SharedFuture future) {
//       bool ok = false;
//       try { auto resp = future.get(); ok = (resp != nullptr); } catch (...) {}
//       auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
//                   std::chrono::steady_clock::now() - t0).count();
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB HAND %s] %s (Δcall=%lldms)",
//         this->ts().c_str(), tag, ok ? "CALLED" : "FAILED", (long long)dt);
//       if (on_done) on_done();
//     }
//   );
// }

// // ---- status callbacks ----
// void MoveSequenceClient::on_right_status(GoalStatusArray::SharedPtr msg)
// {
//   const bool active = any_active(*msg);
//   if (rb3_waiting_) {
//     if (active && !rb3_seen_active_) {
//       rb3_seen_active_ = true;
//       t_active_rb3_ = std::chrono::steady_clock::now();
//       auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_rb3_ - t_send_rb3_).count();
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB3] FIRST ACTIVE (Δsend→active=%lldms)", this->ts().c_str(), (long long)d);
//     }
//     if (rb3_seen_active_ && !active) {
//       rb3_waiting_ = false;
//       auto now = std::chrono::steady_clock::now();
//       auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_rb3_).count();
//       auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_rb3_).count();
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB3] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
//                   this->ts().c_str(), idx_rb3_, (long long)d1, (long long)d2);

//       if (sync_inflight_.has_value() && idx_rb3_ == sync_inflight_->first) {
//         this->run_after_hook_barrier_only(/*is_left=*/false, idx_rb3_);
//         rb3_inflight_done_ = true;
//         this->check_inflight_and_advance_after_both_done();
//         return;
//       }
//       if (this->handle_sync_reached(/*is_left=*/false, idx_rb3_)) return;

//       this->advance_rb3_after_done(idx_rb3_);
//       // 재시작 대기라면 양팔 idle 체크
//       maybe_restart_after_idle();
//     }
//   }
// }

// void MoveSequenceClient::on_left_status(GoalStatusArray::SharedPtr msg)
// {
//   const bool active = any_active(*msg);
//   if (ur3_waiting_) {
//     if (active && !ur3_seen_active_) {
//       ur3_seen_active_ = true;
//       t_active_ur3_ = std::chrono::steady_clock::now();
//       auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_ur3_ - t_send_ur3_).count();
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] FIRST ACTIVE (Δsend→active=%lldms)", this->ts().c_str(), (long long)d);
//     }
//     if (ur3_seen_active_ && !active) {
//       ur3_waiting_ = false;
//       auto now = std::chrono::steady_clock::now();
//       auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_ur3_).count();
//       auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_ur3_).count();
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
//                   this->ts().c_str(), idx_ur3_, (long long)d1, (long long)d2);

//       if (sync_inflight_.has_value() && idx_ur3_ == sync_inflight_->second) {
//         this->run_after_hook_barrier_only(/*is_left=*/true, idx_ur3_);
//         ur3_inflight_done_ = true;
//         this->check_inflight_and_advance_after_both_done();
//         return;
//       }
//       if (this->handle_sync_reached(/*is_left=*/true, idx_ur3_)) return;

//       this->advance_ur3_after_done(idx_ur3_);
//       // 재시작 대기라면 양팔 idle 체크
//       maybe_restart_after_idle();
//     }
//   }
// }

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MoveSequenceClient>());
//   rclcpp::shutdown();
//   return 0;
// }




//--------------------------20251015_양팔다 pick위치에 놓고 재시작_수정중----------------------


#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <optional>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <thread>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

#include "dual_arm_msg/srv/movetcppos_rb3.hpp"
#include "dual_arm_msg/srv/movetcppos_ur3.hpp"
#include "dual_arm_msg/srv/movetcppos.hpp"
#include "inspire_hand_interface/srv/setangle.hpp"

using namespace std::chrono_literals;
using action_msgs::msg::GoalStatusArray;
using action_msgs::msg::GoalStatus;

using RB3Srv = dual_arm_msg::srv::MovetcpposRB3;
using UR3Srv = dual_arm_msg::srv::MovetcpposUR3;
using HandSetAngleSrv = inspire_hand_interface::srv::Setangle;

struct Pose6 { double x, y, z, rx, ry, rz; };
enum class GripperAction { NONE = 0, OPEN, CLOSE, PINCH, HOME};

class MoveSequenceClient : public rclcpp::Node
{
public:
  MoveSequenceClient();

private:
  // ---------- types ----------
  struct PairSync {
    size_t rb3_idx;
    size_t ur3_idx;
    size_t rb3_next;
    size_t ur3_next;
    bool reached_rb3 = false;
    bool reached_ur3 = false;
    bool fired = false;
  };

  // ---------- data ----------
  // active (현재 적용 중) 세트
  std::vector<PairSync> sync_points_;
  std::optional<std::pair<size_t,size_t>> sync_inflight_;
  bool rb3_inflight_done_ = false;
  bool ur3_inflight_done_ = false;

  rclcpp::Client<RB3Srv>::SharedPtr cli_rb3_;
  rclcpp::Client<UR3Srv>::SharedPtr cli_ur3_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_open_, cli_grip_close_;
  rclcpp::Client<HandSetAngleSrv>::SharedPtr cli_hand_setangle_;
  std::string hand_service_name_;

  rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedPtr client_bimanual_tcppos_;
  std::string bimanual_srv_name_;
  bool use_bimanual_srv_ = true;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_pickblock_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_degcheck_;

  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_right_cli_, cancel_left_cli_;

  rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_right_status_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_left_status_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  std::string right_fjt_base_, left_fjt_base_;
  std::vector<Pose6> seq_rb3_, seq_ur3_;
  size_t idx_rb3_ = 0, idx_ur3_ = 0;
  bool rb3_waiting_ = false, ur3_waiting_ = false;
  bool rb3_seen_active_ = false, ur3_seen_active_ = false;
  bool rb3_done_all_ = false,  ur3_done_all_ = false;

  std::unordered_map<size_t, GripperAction> ur3_grip_before_;
  std::unordered_map<size_t, GripperAction> ur3_grip_after_;
  std::unordered_map<size_t, GripperAction> rb3_grip_before_;
  std::unordered_map<size_t, GripperAction> rb3_grip_after_;

  rclcpp::TimerBase::SharedPtr sync_fire_timer_;

  bool hold_until_degcheck_{false};

  std::chrono::steady_clock::time_point t0_steady_;
  std::chrono::steady_clock::time_point t_send_rb3_, t_send_ur3_;
  std::chrono::steady_clock::time_point t_active_rb3_, t_active_ur3_;

  bool exit_when_done_{true};
  bool shutdown_scheduled_{false};
  rclcpp::TimerBase::SharedPtr shutdown_timer_;

  // === (추가) base 스냅샷: 메인 시퀀스/훅/동기점 ===
  std::vector<Pose6> base_seq_rb3_, base_seq_ur3_;
  std::unordered_map<size_t, GripperAction> base_ur3_grip_before_;
  std::unordered_map<size_t, GripperAction> base_ur3_grip_after_;
  std::unordered_map<size_t, GripperAction> base_rb3_grip_before_;
  std::unordered_map<size_t, GripperAction> base_rb3_grip_after_;
  std::vector<PairSync> base_sync_points_;

  // === add this ===
  bool auto_restart_once_ = false;   // A/B 완료 후 1회만 메인 시퀀스로 자동 복귀

  // ---- soft-restart flags ----
  std::atomic<bool> restart_pending_{false};
  bool block_progress_{false};
  rclcpp::TimerBase::SharedPtr restart_watchdog_;

  rclcpp::TimerBase::SharedPtr apply_wait_timer_; // 재시퀀스 적용 대기 타이머

  // ---------- helpers (decl) ----------
  std::string ts() const;
  static bool ends_with(const std::string &s, const std::string &suffix);
  static bool any_active(const GoalStatusArray &arr);
  std::string resolve_status_topic(const std::string &base);

  void start_sequence();
  void reset_sequence_state();
  bool cancel_all_goals();

  // === base 스냅샷/복원 ===
  void snapshot_base_sequences_hooks_sync();
  void restore_base_sequences_hooks_sync();

  void load_sequence_A();  // 조건 충족 시 1회 실행
  void load_sequence_B();  // 조건 미충족 시 1회 실행
  void apply_sequence_and_soft_restart(bool use_A);

  void prepare_and_fire_first_pair();
  void fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks);
  void fire_both_immediate(size_t rb3_idx, size_t ur3_idx);

  void call_zmk_move_to_bimanual_tcppos(double l_x,double l_y,double l_z,double l_rx,double l_ry,double l_rz,
                                        double r_x,double r_y,double r_z,double r_rx,double r_ry,double r_rz);
  void call_zmk_move_to_bimanual_tcppos_async(double l_x,double l_y,double l_z,double l_rx,double l_ry,double l_rz,
                                              double r_x,double r_y,double r_z,double r_rx,double r_ry,double r_rz);

  void start_rb3_step(size_t i);
  void send_rb3_step(size_t i);
  void advance_rb3_after_done(size_t just_finished_idx);

  void start_ur3_step(size_t i);
  void send_ur3_step(size_t i);
  void advance_ur3_after_done(size_t just_finished_idx);

  void run_after_hook_barrier_only(bool is_left, size_t finished_idx);
  bool handle_sync_reached(bool is_left, size_t just_finished_idx);
  bool try_fire_next_sync_after_inflight_pair(size_t just_rb3_idx, size_t just_ur3_idx);
  void check_inflight_and_advance_after_both_done();
  void maybe_finish();

  void request_ur_gripper(GripperAction action, std::function<void()> on_done);
  void request_rb_hand(GripperAction action, std::function<void()> on_done);

  void on_right_status(GoalStatusArray::SharedPtr msg);
  void on_left_status(GoalStatusArray::SharedPtr msg);

  // soft-restart helpers
  void request_soft_restart();
  void maybe_restart_after_idle();
  void do_restart_now();
};

// ===================== IMPLEMENTATION =====================

MoveSequenceClient::MoveSequenceClient()
: Node("move_sequence_client_event_driven")
, t0_steady_(std::chrono::steady_clock::now())
{
  exit_when_done_ = this->declare_parameter<bool>("exit_when_done", true);

  right_fjt_base_ = declare_parameter<std::string>(
    "right_fjt_base", "/joint_trajectory_controller/follow_joint_trajectory");
  left_fjt_base_ = declare_parameter<std::string>(
    "left_fjt_base", "/scaled_joint_trajectory_controller/follow_joint_trajectory");

  cli_rb3_ = this->create_client<RB3Srv>("zmk_tcppos_rb3");
  cli_ur3_ = this->create_client<UR3Srv>("zmk_tcppos_ur3e");

  cli_grip_open_  = this->create_client<std_srvs::srv::Trigger>("zmk_girp_open");
  cli_grip_close_ = this->create_client<std_srvs::srv::Trigger>("zmk_girp_close");

  hand_service_name_ = declare_parameter<std::string>("hand_service_name", "/Setangle");
  cli_hand_setangle_ = this->create_client<HandSetAngleSrv>(hand_service_name_);

  bimanual_srv_name_ = declare_parameter<std::string>("bimanual_srv_name", "zmk_move_to_tcppos");
  use_bimanual_srv_  = declare_parameter<bool>("use_bimanual_srv", true);
  client_bimanual_tcppos_ = this->create_client<dual_arm_msg::srv::Movetcppos>(bimanual_srv_name_);

  cancel_right_cli_ = this->create_client<action_msgs::srv::CancelGoal>(right_fjt_base_ + "/_action/cancel_goal");
  cancel_left_cli_  = this->create_client<action_msgs::srv::CancelGoal>(left_fjt_base_  + "/_action/cancel_goal");

  // ---- 초기 메인 시퀀스/훅/동기점 정의 ----
  seq_rb3_ = {
    {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (0)
    {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (1)
    {-0.04642, -0.44152, 0.120, 1.76208441,    0.24801129,  1.77238186}, // (2)
    { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (3)
    { 0.07944, -0.22198, 0.41277, 1.84917634, -1.4379768,   1.4158111},  // (4)
    { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (5)
    { 0.135,   -0.36851, 0.30087, 0.1446878,   0.1537635,   1.322436},   // (6)
    { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (7)
    {-0.04381, -0.45377, 0.13585, 1.75492856,  0.19338248,  1.8102555},  // (8)
    {-0.0407,  -0.44568, 0.31162, 1.7228145,   0.17715092,  1.80205245}, // (9)
    {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (10)
  };
  seq_ur3_ = {
    { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)
    { -0.140,   -0.410,   0.175,   0.0,   -3.14,  0.0 },      // (1)
    { -0.0800,  -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (2)
    { -0.13056, -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (3)
    { -0.185,   -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (4)
    { -0.13056, -0.32030, 0.46642, 2.151,  0.109, -2.257 },   // (5)
    { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (6)
  };

  // gripper hooks (메인)
  ur3_grip_before_.clear();
  ur3_grip_after_.clear();
  rb3_grip_before_.clear();
  rb3_grip_after_.clear();
  ur3_grip_before_[0] = GripperAction::OPEN;
  ur3_grip_after_[1]  = GripperAction::CLOSE;
  ur3_grip_after_[4]  = GripperAction::OPEN;

  rb3_grip_after_[0]  = GripperAction::OPEN;
  rb3_grip_before_[2] = GripperAction::PINCH;
  rb3_grip_after_[2]  = GripperAction::CLOSE;
  rb3_grip_before_[3] = GripperAction::CLOSE;
  rb3_grip_after_[8]  = GripperAction::PINCH;
  rb3_grip_after_[9]  = GripperAction::HOME;

  // sync points (메인)
  sync_points_.clear();
  sync_points_.push_back(PairSync{/*rb3_idx=*/5, /*ur3_idx=*/3, /*rb3_next=*/6, /*ur3_next=*/4});
  sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/2});

  // === base 스냅샷 저장 (메인으로 복귀 때 사용) ===
  snapshot_base_sequences_hooks_sync();

  // ---- services ----
  srv_degcheck_ = this->create_service<std_srvs::srv::Trigger>(
    "zmk_DegcheckFlag",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
           std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
    {
      if (!hold_until_degcheck_) {
        resp->success = false;
        resp->message = "Not holding at (RB3=4, UR3=2).";
        return;
      }
      hold_until_degcheck_ = false;
      RCLCPP_INFO(this->get_logger(), "[%s] [DEGCHECK] Release → FIRE (RB3=5, UR3=3)", this->ts().c_str());
      idx_rb3_ = 5; idx_ur3_ = 3;
      sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
      rb3_inflight_done_ = ur3_inflight_done_ = false;
      this->fire_both_immediate(idx_rb3_, idx_ur3_);
      resp->success = true;
      resp->message = "Proceed fired to (RB3=5, UR3=3).";
    }
  );
  RCLCPP_INFO(get_logger(), "Service server ready: /zmk_DegcheckFlag");

  // ★ pickblock 조건 서비스
  srv_pickblock_ = this->create_service<std_srvs::srv::Trigger>(
    "zmk_pickblockchkFlag",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
          std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
    {
      const bool use_A = (idx_rb3_ >= 6 && idx_ur3_ >= 4);
      RCLCPP_WARN(this->get_logger(),
        "[%s] [PICKBLOCK] request at RB3=%zu, UR3=%zu → %s",
        this->ts().c_str(), idx_rb3_, idx_ur3_,
        use_A ? "LOAD SEQ-A (once) & RESTART" : "LOAD SEQ-B (once) & RESTART");

      // 이후 진행 잠금
      block_progress_ = true;

      auto apply_now = [this, use_A]() {
        apply_sequence_and_soft_restart(use_A);
      };

      // 이미 양팔 idle이면 즉시 적용
      if (!rb3_waiting_ && !ur3_waiting_) {
        apply_now();
      } else {
        // idle 될 때까지 100ms 폴링
        apply_wait_timer_ = this->create_wall_timer(
          100ms,
          [this, apply_now]() {
            if (!rb3_waiting_ && !ur3_waiting_) {
              if (apply_wait_timer_) apply_wait_timer_->cancel();
              apply_now();
            }
          }
        );
      }

      resp->success = true;
      resp->message = use_A
        ? "Sequence-A will run once then return to MAIN from step 0"
        : "Sequence-B will run once then return to MAIN from step 0";
    }
  );

  RCLCPP_INFO(get_logger(), "Service server ready: /zmk_pickblockchkFlag");

  // bringup timer
  start_timer_ = this->create_wall_timer(300ms, [this] {
    if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s) ||
        !cli_grip_open_->wait_for_service(0s) || !cli_grip_close_->wait_for_service(0s) ||
        !cli_hand_setangle_->wait_for_service(0s) ||
        (use_bimanual_srv_ && !client_bimanual_tcppos_->wait_for_service(0s))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[%s] Waiting for services (rb3/ur3e/gripper/hand%s). hand_service=%s, bimanual=%s",
        this->ts().c_str(), use_bimanual_srv_ ? "/bimanual" : "",
        hand_service_name_.c_str(), bimanual_srv_name_.c_str());
      return;
    }
    const auto right_status = this->resolve_status_topic(right_fjt_base_);
    const auto left_status  = this->resolve_status_topic(left_fjt_base_);
    if (right_status.empty() || left_status.empty()) {
      RCLCPP_ERROR(this->get_logger(),
        "[%s] Could not resolve action status topics. right_base=%s left_base=%s",
        this->ts().c_str(), right_fjt_base_.c_str(), left_fjt_base_.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[%s] [RESOLVED] RIGHT status: %s", this->ts().c_str(), right_status.c_str());
    RCLCPP_INFO(this->get_logger(), "[%s] [RESOLVED] LEFT  status: %s", this->ts().c_str(), left_status.c_str());

    sub_right_status_ = this->create_subscription<GoalStatusArray>(
      right_status, rclcpp::QoS(50),
      std::bind(&MoveSequenceClient::on_right_status, this, std::placeholders::_1));
    sub_left_status_ = this->create_subscription<GoalStatusArray>(
      left_status, rclcpp::QoS(50),
      std::bind(&MoveSequenceClient::on_left_status, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[%s] Services ready. Using hand_service=%s, bimanual=%s (use=%s)",
                this->ts().c_str(), hand_service_name_.c_str(), bimanual_srv_name_.c_str(),
                use_bimanual_srv_ ? "true" : "false");

    start_timer_->cancel();
    this->start_sequence();
  });
}

// ---- helpers ----
std::string MoveSequenceClient::ts() const {
  using namespace std::chrono;
  auto now = steady_clock::now();
  auto ms  = duration_cast<milliseconds>(now - t0_steady_).count();
  std::ostringstream oss; oss << std::setw(9) << ms << "ms";
  return oss.str();
}
bool MoveSequenceClient::ends_with(const std::string &s, const std::string &suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}
bool MoveSequenceClient::any_active(const GoalStatusArray &arr) {
  for (const auto &st : arr.status_list) {
    if (st.status == GoalStatus::STATUS_ACCEPTED ||
        st.status == GoalStatus::STATUS_EXECUTING ||
        st.status == GoalStatus::STATUS_CANCELING) return true;
  }
  return false;
}
std::string MoveSequenceClient::resolve_status_topic(const std::string &base)
{
  const auto a = base + "/_action/status";
  const auto b = base + "/status";
  auto graph = this->get_topic_names_and_types();
  if (graph.find(a) != graph.end()) return a;
  if (graph.find(b) != graph.end()) return b;
  for (const auto &kv : graph) {
    const auto &topic = kv.first;
    if ((ends_with(topic, "/_action/status") || ends_with(topic, "/status")) &&
        topic.find(base) != std::string::npos) {
      return topic;
    }
  }
  return std::string();
}

// ---- base 스냅샷/복원 ----
void MoveSequenceClient::snapshot_base_sequences_hooks_sync()
{
  base_seq_rb3_ = seq_rb3_;
  base_seq_ur3_ = seq_ur3_;
  base_ur3_grip_before_ = ur3_grip_before_;
  base_ur3_grip_after_  = ur3_grip_after_;
  base_rb3_grip_before_ = rb3_grip_before_;
  base_rb3_grip_after_  = rb3_grip_after_;
  base_sync_points_     = sync_points_;
  RCLCPP_INFO(this->get_logger(), "[%s] [BASE] snapshot stored.", this->ts().c_str());
}

void MoveSequenceClient::restore_base_sequences_hooks_sync()
{
  seq_rb3_ = base_seq_rb3_;
  seq_ur3_ = base_seq_ur3_;
  ur3_grip_before_ = base_ur3_grip_before_;
  ur3_grip_after_  = base_ur3_grip_after_;
  rb3_grip_before_ = base_rb3_grip_before_;
  rb3_grip_after_  = base_rb3_grip_after_;
  sync_points_     = base_sync_points_;
  RCLCPP_INFO(this->get_logger(), "[%s] [BASE] restored MAIN sequences/hooks/sync.", this->ts().c_str());
}

// ---- restart/cancel ----
void MoveSequenceClient::start_sequence() {
  RCLCPP_INFO(this->get_logger(), "[%s] === Sequence PREP ===", this->ts().c_str());
  block_progress_ = false;
  this->prepare_and_fire_first_pair();
}
void MoveSequenceClient::reset_sequence_state() {
  rb3_waiting_ = ur3_waiting_ = false;
  rb3_seen_active_ = ur3_seen_active_ = false;
  rb3_done_all_ = ur3_done_all_ = false;
  idx_rb3_ = idx_ur3_ = 0;
  hold_until_degcheck_ = false;
  for (auto &sp : sync_points_) { sp.reached_rb3=false; sp.reached_ur3=false; sp.fired=false; }
  sync_inflight_.reset();
  if (sync_fire_timer_) sync_fire_timer_->cancel();
  RCLCPP_INFO(this->get_logger(), "[%s] [RESET] sequence state cleared.", this->ts().c_str());
}
bool MoveSequenceClient::cancel_all_goals()
{
  if (!cancel_right_cli_->wait_for_service(500ms) ||
      !cancel_left_cli_->wait_for_service(500ms)) {
    RCLCPP_ERROR(this->get_logger(), "[%s] CancelGoal service not available", this->ts().c_str());
    return false;
  }
  auto make_cancel_req = [](){
    auto req = std::make_shared<action_msgs::srv::CancelGoal::Request>();
    for (auto &b : req->goal_info.goal_id.uuid) b = 0;
    req->goal_info.stamp.sec = 0;
    req->goal_info.stamp.nanosec = 0;
    return req;
  };
  auto f_r = cancel_right_cli_->async_send_request(make_cancel_req());
  auto f_l = cancel_left_cli_->async_send_request(make_cancel_req());

  bool ok_r = (f_r.wait_for(1s) == std::future_status::ready);
  bool ok_l = (f_l.wait_for(1s) == std::future_status::ready);
  if (!ok_r || !ok_l) {
    RCLCPP_ERROR(this->get_logger(), "[%s] CancelGoal timeout (right=%s left=%s)",
                 this->ts().c_str(), ok_r?"ok":"timeout", ok_l?"ok":"timeout");
    return false;
  }
  try {
    auto r = f_r.get(); auto l = f_l.get();
    RCLCPP_INFO(this->get_logger(), "[%s] CancelGoal ret: right(code=%d) left(code=%d)",
                this->ts().c_str(), r->return_code, l->return_code);
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "[%s] CancelGoal exception (ignored)", this->ts().c_str());
  }
  std::this_thread::sleep_for(100ms);
  return true;
}

void MoveSequenceClient::apply_sequence_and_soft_restart(bool use_A)
{
  if (use_A) load_sequence_A();
  else       load_sequence_B();

  // A/B는 1회만 실행하고 종료되면 메인으로 복귀
  auto_restart_once_ = true;
  exit_when_done_ = false;

  // 상태 초기화 후 0번부터 재시작 (A/B를 active로)
  this->reset_sequence_state();
  block_progress_ = false;
  this->start_sequence();
}

void MoveSequenceClient::load_sequence_A()
{
  // === 새 시퀀스 A (간단 예시) ===
  seq_rb3_.assign({
    {-0.04381, -0.45377, 0.13585, 1.75492856,  0.19338248,  1.8102555},  // (0)
    {-0.0407,  -0.44568, 0.31162, 1.7228145,   0.17715092,  1.80205245}, // (1)
    {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (2)
  });
  seq_ur3_.assign({
    { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)
    { -0.140,   -0.410,   0.175,   0.0,   -3.14,  0.0 },      // (1)
    { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (2)
  });

  // 훅 초기화 (A에서는 훅/동기점 사용 안 함)
  ur3_grip_before_.clear();
  ur3_grip_after_.clear();
  rb3_grip_before_.clear();
  rb3_grip_after_.clear();

  ur3_grip_after_[1]  = GripperAction::OPEN;
  rb3_grip_after_[0]  = GripperAction::PINCH;
  rb3_grip_after_[1]  = GripperAction::HOME;

  sync_points_.clear();

  RCLCPP_INFO(this->get_logger(), "[%s] SEQ-A loaded (one-shot).", this->ts().c_str());
}

void MoveSequenceClient::load_sequence_B()
{
  // === 새 시퀀스 B (간단 예시) ===
  seq_rb3_.assign({
    {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (0)
    {-0.04642, -0.44152, 0.121, 1.76208441,  0.24801129,  1.77238186}, // (1)
    {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (2)
    {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (3)
  });
  seq_ur3_.assign({
    { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)
    { -0.140,   -0.410,   0.175,   0.0,   -3.14,  0.0 },      // (1)
    { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (2)
  });

  ur3_grip_before_.clear();
  ur3_grip_after_.clear();
  rb3_grip_before_.clear();
  rb3_grip_after_.clear();

  // 예시로 간단 훅만
  ur3_grip_after_[1]  = GripperAction::OPEN;
  rb3_grip_after_[1]  = GripperAction::PINCH;
  rb3_grip_after_[2]  = GripperAction::PINCH;
  rb3_grip_after_[3]  = GripperAction::HOME;

  sync_points_.clear();

  RCLCPP_WARN(this->get_logger(), "[%s] SEQ-B loaded (one-shot).", this->ts().c_str());
}

// ---- soft-restart helpers ----
void MoveSequenceClient::request_soft_restart()
{
  restart_pending_.store(true);
  block_progress_ = true;

  if (!restart_watchdog_ || restart_watchdog_->is_canceled()) {
    restart_watchdog_ = this->create_wall_timer(
      100ms, [this](){
        if (!restart_pending_.load()) { restart_watchdog_->cancel(); return; }
        if (!rb3_waiting_ && !ur3_waiting_) {
          RCLCPP_INFO(this->get_logger(), "[%s] [RESTART WD] both idle → restart now", this->ts().c_str());
          restart_watchdog_->cancel();
          do_restart_now();
        }
      }
    );
  }
  maybe_restart_after_idle();
}

void MoveSequenceClient::maybe_restart_after_idle()
{
  if (restart_pending_.load() && !rb3_waiting_ && !ur3_waiting_) {
    do_restart_now();
  }
}

void MoveSequenceClient::do_restart_now()
{
  if (!restart_pending_.load()) return;
  RCLCPP_WARN(this->get_logger(), "[%s] [PICKBLOCK] Soft-restart NOW → reset & start from 0", this->ts().c_str());
  this->reset_sequence_state();
  restart_pending_.store(false);
  block_progress_ = false;
  this->start_sequence();
}

// ---- first pair ----
void MoveSequenceClient::prepare_and_fire_first_pair() {
  if (restart_pending_.load()) {
    RCLCPP_INFO(this->get_logger(), "[%s] Soft-restart pending → not firing first pair", this->ts().c_str());
    return;
  }

  const bool need_rb3 = rb3_grip_before_.count(0) && rb3_grip_before_[0] != GripperAction::NONE;
  const bool need_ur3 = ur3_grip_before_.count(0) && ur3_grip_before_[0] != GripperAction::NONE;

  if (!need_rb3 && !need_ur3) {
    RCLCPP_INFO(this->get_logger(), "[%s] No BEFORE hooks at idx0 → FIRE BOTH step0 now", this->ts().c_str());
    sync_inflight_ = std::make_pair(0u, 0u);
    rb3_inflight_done_ = ur3_inflight_done_ = false;
    this->fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks at idx0: RB3=%s, UR3=%s",
    this->ts().c_str(), need_rb3 ? "YES" : "NO", need_ur3 ? "YES" : "NO");

  auto pending = std::make_shared<std::atomic<int>>(0);
  if (need_rb3) pending->fetch_add(1);
  if (need_ur3) pending->fetch_add(1);

  auto after = [this, pending]() {
    if (pending->fetch_sub(1) == 1) {
      RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks done → FIRE BOTH step0", this->ts().c_str());
      sync_inflight_ = std::make_pair(0u, 0u);
      rb3_inflight_done_ = ur3_inflight_done_ = false;
      this->fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
    }
  };

  if (need_rb3) {
    RCLCPP_INFO(this->get_logger(), "[%s] [PREP] RB3 BEFORE hook (idx0)", this->ts().c_str());
    this->request_rb_hand(rb3_grip_before_[0], after);
  }
  if (need_ur3) {
    RCLCPP_INFO(this->get_logger(), "[%s] [PREP] UR3e BEFORE hook (idx0)", this->ts().c_str());
    this->request_ur_gripper(ur3_grip_before_[0], after);
  }
}

// ---- concurrent fire ----
void MoveSequenceClient::fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks)
{
  if (restart_pending_.load()) {
    RCLCPP_INFO(this->get_logger(), "[%s] Soft-restart pending → skip FIRE BOTH", this->ts().c_str());
    return;
  }

  if (!apply_before_hooks) {
    sync_fire_timer_ = this->create_wall_timer(
      0ms,
      [this, rb3_idx, ur3_idx](){
        if (restart_pending_.load()) { sync_fire_timer_->cancel(); return; }
        RCLCPP_INFO(this->get_logger(),
          "[%s] FIRE BOTH NOW (RB3=%zu, UR3e=%zu) — same tick",
          this->ts().c_str(), rb3_idx, ur3_idx);

        if (use_bimanual_srv_) {
          const auto &r = seq_rb3_[rb3_idx];
          const auto &l = seq_ur3_[ur3_idx];
          this->call_zmk_move_to_bimanual_tcppos_async(
            /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
            /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
        } else {
          this->send_ur3_step(ur3_idx);
          this->send_rb3_step(rb3_idx);
        }
        sync_fire_timer_->cancel();
      }
    );
    return;
  }

  size_t pending = 0;
  bool do_rb3_before = rb3_grip_before_.count(rb3_idx) && rb3_grip_before_[rb3_idx] != GripperAction::NONE;
  bool do_ur3_before = ur3_grip_before_.count(ur3_idx) && ur3_grip_before_[ur3_idx] != GripperAction::NONE;

  auto after_one_done = [this, &pending, rb3_idx, ur3_idx]() mutable {
    if (--pending == 0) {
      sync_fire_timer_ = this->create_wall_timer(
        0ms,
        [this, rb3_idx, ur3_idx](){
          if (restart_pending_.load()) { sync_fire_timer_->cancel(); return; }
          RCLCPP_INFO(this->get_logger(),
            "[%s] FIRE BOTH (after hooks) (RB3=%zu, UR3e=%zu)", this->ts().c_str(), rb3_idx, ur3_idx);

          if (use_bimanual_srv_) {
            const auto &r = seq_rb3_[rb3_idx];
            const auto &l = seq_ur3_[ur3_idx];
            this->call_zmk_move_to_bimanual_tcppos_async(
              /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
              /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
          } else {
            this->send_rb3_step(rb3_idx);
            this->send_ur3_step(ur3_idx);
          }
          sync_fire_timer_->cancel();
        }
      );
    }
  };

  if (do_rb3_before) pending++;
  if (do_ur3_before) pending++;
  if (pending == 0) {
    sync_fire_timer_ = this->create_wall_timer(
      0ms, [this, rb3_idx, ur3_idx](){
        if (restart_pending_.load()) { sync_fire_timer_->cancel(); return; }
        RCLCPP_INFO(this->get_logger(),
          "[%s] FIRE BOTH (no hooks) (RB3=%zu, UR3e=%zu)", this->ts().c_str(), rb3_idx, ur3_idx);
        if (use_bimanual_srv_) {
          const auto &r = seq_rb3_[rb3_idx];
          const auto &l = seq_ur3_[ur3_idx];
          this->call_zmk_move_to_bimanual_tcppos_async(
            /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
            /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
        } else {
          this->send_rb3_step(rb3_idx);
          this->send_ur3_step(ur3_idx);
        }
        sync_fire_timer_->cancel();
      }
    );
    return;
  }
  if (do_rb3_before) {
    RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] RB3 BEFORE hook (idx%zu)", this->ts().c_str(), rb3_idx);
    this->request_rb_hand(rb3_grip_before_[rb3_idx], [after_one_done](){});
  }
  if (do_ur3_before) {
    RCLCPP_INFO(this->get_logger(), "[%s] [BARRIER] UR3e BEFORE hook (idx%zu)", this->ts().c_str(), ur3_idx);
    this->request_ur_gripper(ur3_grip_before_[ur3_idx], [after_one_done](){});
  }
}

void MoveSequenceClient::fire_both_immediate(size_t rb3_idx, size_t ur3_idx)
{
  if (restart_pending_.load()) {
    RCLCPP_INFO(this->get_logger(), "[%s] Soft-restart pending → skip FIRE BOTH IMMEDIATE", this->ts().c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(),
    "[%s] FIRE BOTH IMMEDIATE (RB3=%zu, UR3e=%zu)", this->ts().c_str(), rb3_idx, ur3_idx);

  if (use_bimanual_srv_) {
    const auto &r = seq_rb3_[rb3_idx];
    const auto &l = seq_ur3_[ur3_idx];
    this->call_zmk_move_to_bimanual_tcppos_async(
      /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
      /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
  } else {
    this->send_ur3_step(ur3_idx);
    this->send_rb3_step(rb3_idx);
  }
}

// ---- bimanual calls ----
void MoveSequenceClient::call_zmk_move_to_bimanual_tcppos(
  double l_x,double l_y,double l_z,double l_rx,double l_ry,double l_rz,
  double r_x,double r_y,double r_z,double r_rx,double r_ry,double r_rz)
{
  auto req = std::make_shared<dual_arm_msg::srv::Movetcppos::Request>();
  req->left_x = l_x;  req->left_y = l_y;  req->left_z = l_z;
  req->left_rx = l_rx; req->left_ry = l_ry; req->left_rz = l_rz;
  req->right_x = r_x; req->right_y = r_y; req->right_z = r_z;
  req->right_rx = r_rx; req->right_ry = r_ry; req->right_rz = r_rz;

  if (!client_bimanual_tcppos_->wait_for_service(2s)) {
    RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] service not available: %s", bimanual_srv_name_.c_str());
    return;
  }
  auto future = client_bimanual_tcppos_->async_send_request(req);
  auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s);
  if (ret != rclcpp::FutureReturnCode::SUCCESS || !future.valid()) {
    RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] service timeout/invalid");
    return;
  }
  const auto resp = future.get();
  if (resp && resp->success) {
    RCLCPP_INFO(this->get_logger(), "[BIMANUAL] move_tcppos success");
  } else {
    RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] move_tcppos failed: %s",
                 resp ? resp->message.c_str() : "no response");
  }
}

void MoveSequenceClient::call_zmk_move_to_bimanual_tcppos_async(
  double l_x,double l_y,double l_z,double l_rx,double l_ry,double l_rz,
  double r_x,double r_y,double r_z,double r_rx,double r_ry,double r_rz)
{
  auto req = std::make_shared<dual_arm_msg::srv::Movetcppos::Request>();
  req->left_x = l_x;  req->left_y = l_y;  req->left_z = l_z;
  req->left_rx = l_rx; req->left_ry = l_ry; req->left_rz = l_rz;
  req->right_x = r_x; req->right_y = r_y; req->right_z = r_z;
  req->right_rx = r_rx; req->right_ry = r_ry; req->right_rz = r_rz;

  rb3_waiting_ = ur3_waiting_ = true;
  rb3_seen_active_ = ur3_seen_active_ = false;
  t_send_rb3_ = t_send_ur3_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(),
    "[%s] [BIMANUAL] SEND tcppos (UR3e & RB3) via %s", this->ts().c_str(), bimanual_srv_name_.c_str());

  client_bimanual_tcppos_->async_send_request(
    req,
    [this](rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedFuture f){
      bool ok = false; std::string msg = "no response";
      try { auto resp = f.get(); ok = resp && resp->success; if (resp) msg = resp->message; }
      catch (const std::exception& e) { msg = e.what(); }
      catch (...) {}
      RCLCPP_INFO(this->get_logger(), "[%s] [BIMANUAL] request SENT %s (%s)",
                  this->ts().c_str(), ok ? "OK" : "FAILED", msg.c_str());
    });
}

// ---- RB3 ----
void MoveSequenceClient::start_rb3_step(size_t i)
{
  if (i >= seq_rb3_.size()) { if (!rb3_done_all_) { rb3_done_all_ = true; this->maybe_finish(); } return; }
  if (rb3_grip_before_.count(i) && rb3_grip_before_[i] != GripperAction::NONE) {
    RCLCPP_INFO(this->get_logger(), "[%s] [RB3] BEFORE hook at idx=%zu", this->ts().c_str(), i);
    this->request_rb_hand(rb3_grip_before_[i], /*on_done=*/nullptr);
    this->send_rb3_step(i);
  } else {
    this->send_rb3_step(i);
  }
}
void MoveSequenceClient::send_rb3_step(size_t i)
{
  if (i >= seq_rb3_.size()) return;
  const auto &p = seq_rb3_[i];
  auto req = std::make_shared<RB3Srv::Request>();
  req->right_x = p.x; req->right_y = p.y; req->right_z = p.z;
  req->right_rx = p.rx; req->right_ry = p.ry; req->right_rz = p.rz;

  rb3_waiting_ = true;
  rb3_seen_active_ = false;
  t_send_rb3_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(), "[%s] [RB3] SEND step %zu", this->ts().c_str(), i);
  cli_rb3_->async_send_request(
    req,
    [this, i](rclcpp::Client<RB3Srv>::SharedFuture future) {
      auto resp = future.get();
      if (resp && resp->success) {
        RCLCPP_INFO(this->get_logger(), "[%s] [RB3] step %zu SENT OK: %s",
                    this->ts().c_str(), i, resp->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "[%s] [RB3] step %zu SEND FAILED", this->ts().c_str(), i);
      }
    });
}
void MoveSequenceClient::advance_rb3_after_done(size_t just_finished_idx)
{
  if (restart_pending_.load() || block_progress_) {
    rb3_waiting_ = false;
    maybe_restart_after_idle();
    return;
  }
  if (rb3_grip_after_.count(just_finished_idx) &&
      rb3_grip_after_[just_finished_idx] != GripperAction::NONE) {
    RCLCPP_INFO(this->get_logger(), "[%s] [RB3] AFTER hook at idx=%zu", this->ts().c_str(), just_finished_idx);
    this->request_rb_hand(rb3_grip_after_[just_finished_idx], [this]{ ++idx_rb3_; this->start_rb3_step(idx_rb3_); });
  } else {
    ++idx_rb3_;
    this->start_rb3_step(idx_rb3_);
  }
}

// ---- UR3e ----
void MoveSequenceClient::start_ur3_step(size_t i)
{
  if (i >= seq_ur3_.size()) { if (!ur3_done_all_) { ur3_done_all_ = true; this->maybe_finish(); } return; }
  if (ur3_grip_before_.count(i) && ur3_grip_before_[i] != GripperAction::NONE) {
    RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] BEFORE hook at idx=%zu", this->ts().c_str(), i);
    this->request_ur_gripper(ur3_grip_before_[i], [this, i]{ this->send_ur3_step(i); });
  } else {
    this->send_ur3_step(i);
  }
}
void MoveSequenceClient::send_ur3_step(size_t i)
{
  if (i >= seq_ur3_.size()) return;
  const auto &p = seq_ur3_[i];
  auto req = std::make_shared<UR3Srv::Request>();
  req->left_x = p.x; req->left_y = p.y; req->left_z = p.z;
  req->left_rx = p.rx; req->left_ry = p.ry; req->left_rz = p.rz;

  ur3_waiting_ = true;
  ur3_seen_active_ = false;
  t_send_ur3_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] SEND step %zu", this->ts().c_str(), i);
  cli_ur3_->async_send_request(
    req,
    [this, i](rclcpp::Client<UR3Srv>::SharedFuture future) {
      auto resp = future.get();
      if (resp && resp->success) {
        RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] step %zu SENT OK: %s",
                    this->ts().c_str(), i, resp->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "[%s] [UR3e] step %zu SEND FAILED", this->ts().c_str(), i);
      }
    });
}
void MoveSequenceClient::advance_ur3_after_done(size_t just_finished_idx)
{
  if (restart_pending_.load() || block_progress_) {
    ur3_waiting_ = false;
    maybe_restart_after_idle();
    return;
  }
  if (ur3_grip_after_.count(just_finished_idx) &&
      ur3_grip_after_[just_finished_idx] != GripperAction::NONE) {
    RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] AFTER hook at idx=%zu", this->ts().c_str(), just_finished_idx);
    this->request_ur_gripper(ur3_grip_after_[just_finished_idx], [this]{ ++idx_ur3_; this->start_ur3_step(idx_ur3_); });
  } else {
    ++idx_ur3_;
    this->start_ur3_step(idx_ur3_);
  }
}

// ---- sync / inflight ----
void MoveSequenceClient::run_after_hook_barrier_only(bool is_left, size_t finished_idx)
{
  GripperAction act = GripperAction::NONE;
  if (is_left) {
    auto it = ur3_grip_after_.find(finished_idx);
    if (it != ur3_grip_after_.end()) act = it->second;
  } else {
    auto it = rb3_grip_after_.find(finished_idx);
    if (it != rb3_grip_after_.end()) act = it->second;
  }
  if (act != GripperAction::NONE) {
    RCLCPP_INFO(this->get_logger(),
      "[%s] [BARRIER AFTER] %s idx=%zu → %s",
      this->ts().c_str(), is_left ? "UR3e" : "RB3", finished_idx,
      act == GripperAction::OPEN ? "OPEN" : (act == GripperAction::CLOSE ? "CLOSE" : "PINCH"));
    if (is_left) this->request_ur_gripper(act, /*on_done=*/nullptr);
    else         this->request_rb_hand(act,    /*on_done=*/nullptr);
  }
}

bool MoveSequenceClient::handle_sync_reached(bool is_left, size_t just_finished_idx)
{
  bool matched_any = false;
  for (auto &sp : sync_points_) {
    if (sp.fired) continue;

    if (!is_left && just_finished_idx == sp.rb3_idx) {
      sp.reached_rb3 = true; matched_any = true;
      RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] RB3 reached idx=%zu", this->ts().c_str(), sp.rb3_idx);
      this->run_after_hook_barrier_only(/*is_left=*/false, just_finished_idx);
    }
    if ( is_left && just_finished_idx == sp.ur3_idx) {
      sp.reached_ur3 = true; matched_any = true;
      RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] UR3e reached idx=%zu", this->ts().c_str(), sp.ur3_idx);
      this->run_after_hook_barrier_only(/*is_left=*/true, just_finished_idx);
    }

    if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
      sp.fired = true;
      if (restart_pending_.load() || block_progress_) {
        RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] Soft-restart pending → not firing next pair", this->ts().c_str());
        return true;
      }
      idx_rb3_ = sp.rb3_next;
      idx_ur3_ = sp.ur3_next;

      sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
      rb3_inflight_done_ = ur3_inflight_done_ = false;

      RCLCPP_INFO(this->get_logger(),
        "[%s] [SYNC] BOTH reached → FIRE NOW (RB3=%zu, UR3e=%zu)",
        this->ts().c_str(), idx_rb3_, idx_ur3_);

      this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
      return true;
    }
  }
  return matched_any;
}

bool MoveSequenceClient::try_fire_next_sync_after_inflight_pair(size_t just_rb3_idx, size_t just_ur3_idx)
{
  for (auto &sp : sync_points_) {
    if (sp.rb3_idx == just_rb3_idx && sp.ur3_idx == just_ur3_idx) {
      if (sp.fired) return false;
      sp.fired = true;

      if (restart_pending_.load() || block_progress_) {
        RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] Soft-restart pending → not firing next pair", this->ts().c_str());
        return true;
      }

      idx_rb3_ = sp.rb3_next;
      idx_ur3_ = sp.ur3_next;

      sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
      rb3_inflight_done_ = ur3_inflight_done_ = false;

      RCLCPP_INFO(this->get_logger(),
        "[%s] [SYNC/POST-INFLIGHT] (RB3=%zu, UR3=%zu) → FIRE NEXT (RB3=%zu, UR3=%zu)",
        this->ts().c_str(), just_rb3_idx, just_ur3_idx, idx_rb3_, idx_ur3_);

      this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
      return true;
    }
  }
  return false;
}

void MoveSequenceClient::check_inflight_and_advance_after_both_done()
{
  if (!sync_inflight_.has_value()) return;
  if (!(rb3_inflight_done_ && ur3_inflight_done_)) return;

  auto [rb3_idx, ur3_idx] = *sync_inflight_;
  RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] inflight done (RB3=%zu, UR3e=%zu)", this->ts().c_str(), rb3_idx, ur3_idx);

  if (rb3_idx == 4 && ur3_idx == 2) {
    hold_until_degcheck_ = true;
    sync_inflight_.reset();
    RCLCPP_INFO(this->get_logger(), "[%s] [DEGCHECK] Holding at (RB3=4, UR3=2). Call `/zmk_DegcheckFlag`.", this->ts().c_str());
    return;
  }

  if (try_fire_next_sync_after_inflight_pair(rb3_idx, ur3_idx)) {
    return;
  }

  sync_inflight_.reset();
  rb3_inflight_done_ = ur3_inflight_done_ = false;

  if (restart_pending_.load() || block_progress_) {
    RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] Soft-restart pending → hold after inflight", this->ts().c_str());
    rb3_waiting_ = false;
    ur3_waiting_ = false;
    maybe_restart_after_idle();
    return;
  }

  ++idx_rb3_;  this->start_rb3_step(idx_rb3_);
  ++idx_ur3_;  this->start_ur3_step(idx_ur3_);
}

void MoveSequenceClient::maybe_finish() {
  if (rb3_done_all_ && ur3_done_all_) {
    if (auto_restart_once_) {
      RCLCPP_INFO(this->get_logger(),
        "[%s] One-shot sequence finished → RESTORE MAIN & restart from step 0.", this->ts().c_str());
      auto_restart_once_ = false;   // 1회만
      // 메인 시퀀스로 복귀
      restore_base_sequences_hooks_sync();
      this->reset_sequence_state();
      this->start_sequence();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[%s] === Sequence ALL DONE ===", this->ts().c_str());
    if (exit_when_done_ && !shutdown_scheduled_) {
      shutdown_scheduled_ = true;
      shutdown_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() {
          RCLCPP_INFO(this->get_logger(), "[%s] Shutting down (exit_when_done=true).", this->ts().c_str());
          rclcpp::shutdown();
        });
    }
  }
}

// ---- services to grippers ----
void MoveSequenceClient::request_ur_gripper(GripperAction action, std::function<void()> on_done)
{
  if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto cli = (action == GripperAction::CLOSE) ? cli_grip_close_ : cli_grip_open_;
  if (!cli->wait_for_service(0s)) {
    RCLCPP_WARN(this->get_logger(), "[%s] UR gripper service not available (%s).",
                this->ts().c_str(), action == GripperAction::CLOSE ? "CLOSE" : "OPEN");
    if (on_done) on_done();
    return;
  }
  auto t0 = std::chrono::steady_clock::now();
  cli->async_send_request(
    req,
    [this, action, on_done, t0](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      bool ok = false;
      try { auto resp = future.get(); ok = resp && resp->success; } catch (...) {}
      auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(this->get_logger(), "[%s] [UR %s] %s (Δcall=%lldms)",
        this->ts().c_str(), (action == GripperAction::CLOSE ? "CLOSE" : "OPEN"), ok ? "OK" : "FAILED", (long long)dt);
      if (on_done) on_done();
    });
}

void MoveSequenceClient::request_rb_hand(GripperAction action, std::function<void()> on_done)
{
  if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
  if (!cli_hand_setangle_->wait_for_service(0s)) {
    RCLCPP_WARN(this->get_logger(), "[%s] RB hand service not available: %s",
                this->ts().c_str(), hand_service_name_.c_str());
    if (on_done) on_done();
    return;
  }

  auto req = std::make_shared<HandSetAngleSrv::Request>();
  if (action == GripperAction::CLOSE)
  {
    req->angle0 = 0; req->angle1 = 0; req->angle2 = 645;
    req->angle3 = 671;  req->angle4 = 260;  req->angle5 = 0;
  }
  else if(action == GripperAction::PINCH)
  {
    req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
    req->angle3 = 1000; req->angle4 = 400;  req->angle5 = 0;
  }
  else if(action == GripperAction::HOME)
  {
    req->angle0 = 0; req->angle1 = 0; req->angle2 = 0;
    req->angle3 = 0; req->angle4 = 0;  req->angle5 = 1000;
  }
  else
  { // OPEN
    req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
    req->angle3 = 1000; req->angle4 = 0; req->angle5 = 1000;
  }
  req->hand_id = 1;
  req->status  = "set_angle";

  const char* tag = (action == GripperAction::CLOSE ? "CLOSE" :
                    action == GripperAction::OPEN  ? "OPEN"  :
                    action == GripperAction::PINCH ? "PINCH" : "HOME");

  auto t0 = std::chrono::steady_clock::now();
  cli_hand_setangle_->async_send_request(
    req,
    [this, on_done, t0, tag](rclcpp::Client<HandSetAngleSrv>::SharedFuture future) {
      bool ok = false;
      try { auto resp = future.get(); ok = (resp != nullptr); } catch (...) {}
      auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(this->get_logger(), "[%s] [RB HAND %s] %s (Δcall=%lldms)",
        this->ts().c_str(), tag, ok ? "CALLED" : "FAILED", (long long)dt);
      if (on_done) on_done();
    }
  );
}

// ---- status callbacks ----
void MoveSequenceClient::on_right_status(GoalStatusArray::SharedPtr msg)
{
  const bool active = any_active(*msg);
  if (rb3_waiting_) {
    if (active && !rb3_seen_active_) {
      rb3_seen_active_ = true;
      t_active_rb3_ = std::chrono::steady_clock::now();
      auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_rb3_ - t_send_rb3_).count();
      RCLCPP_INFO(this->get_logger(), "[%s] [RB3] FIRST ACTIVE (Δsend→active=%lldms)", this->ts().c_str(), (long long)d);
    }
    if (rb3_seen_active_ && !active) {
      rb3_waiting_ = false;
      auto now = std::chrono::steady_clock::now();
      auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_rb3_).count();
      auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_rb3_).count();
      RCLCPP_INFO(this->get_logger(), "[%s] [RB3] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
                  this->ts().c_str(), idx_rb3_, (long long)d1, (long long)d2);

      if (sync_inflight_.has_value() && idx_rb3_ == sync_inflight_->first) {
        this->run_after_hook_barrier_only(/*is_left=*/false, idx_rb3_);
        rb3_inflight_done_ = true;
        this->check_inflight_and_advance_after_both_done();
        return;
      }
      if (this->handle_sync_reached(/*is_left=*/false, idx_rb3_)) return;

      this->advance_rb3_after_done(idx_rb3_);
      maybe_restart_after_idle();
    }
  }
}

void MoveSequenceClient::on_left_status(GoalStatusArray::SharedPtr msg)
{
  const bool active = any_active(*msg);
  if (ur3_waiting_) {
    if (active && !ur3_seen_active_) {
      ur3_seen_active_ = true;
      t_active_ur3_ = std::chrono::steady_clock::now();
      auto d = std::chrono::duration_cast<std::chrono::milliseconds>(t_active_ur3_ - t_send_ur3_).count();
      RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] FIRST ACTIVE (Δsend→active=%lldms)", this->ts().c_str(), (long long)d);
    }
    if (ur3_seen_active_ && !active) {
      ur3_waiting_ = false;
      auto now = std::chrono::steady_clock::now();
      auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_ur3_).count();
      auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_ur3_).count();
      RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] DONE at idx=%zu (Δsend→done=%lldms, Δactive→done=%lldms)",
                  this->ts().c_str(), idx_ur3_, (long long)d1, (long long)d2);

      if (sync_inflight_.has_value() && idx_ur3_ == sync_inflight_->second) {
        this->run_after_hook_barrier_only(/*is_left=*/true, idx_ur3_);
        ur3_inflight_done_ = true;
        this->check_inflight_and_advance_after_both_done();
        return;
      }
      if (this->handle_sync_reached(/*is_left=*/true, idx_ur3_)) return;

      this->advance_ur3_after_done(idx_ur3_);
      maybe_restart_after_idle();
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveSequenceClient>());
  rclcpp::shutdown();
  return 0;
}
