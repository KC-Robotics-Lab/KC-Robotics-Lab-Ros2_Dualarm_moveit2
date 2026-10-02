// file: move_sequence_client_with_servo.cpp
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
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <action_msgs/msg/goal_status_array.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
enum class GripperAction { NONE = 0, OPEN, CLOSE, PINCH };

class MoveSequenceClient : public rclcpp::Node
{
public:
  MoveSequenceClient()
  : Node("move_sequence_client_event_driven_servo")
  , t0_steady_(std::chrono::steady_clock::now())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  {
    // ===== Parameters =====
    exit_when_done_ = this->declare_parameter<bool>("exit_when_done", true);

    right_fjt_base_ = declare_parameter<std::string>("right_fjt_base",
      "/joint_trajectory_controller/follow_joint_trajectory");
    left_fjt_base_  = declare_parameter<std::string>("left_fjt_base",
      "/scaled_joint_trajectory_controller/follow_joint_trajectory");

    // Servo topics/frames
    left_servo_twist_topic_  = declare_parameter<std::string>("left_servo_twist_topic",
      "/left/servo_node/delta_twist_cmds");
    right_servo_twist_topic_ = declare_parameter<std::string>("right_servo_twist_topic",
      "/right/servo_node/delta_twist_cmds");
    camera_pose_topic_ = declare_parameter<std::string>("camera_pose_topic", "/camera/target_tcp");
    camera_frame_      = declare_parameter<std::string>("camera_frame", "camera_frame");

    // Command frame for Servo (MoveIt Servo 설정 참조: usually "base" or "tool")
    left_base_frame_   = declare_parameter<std::string>("left_base_frame",  "left_base");
    right_base_frame_  = declare_parameter<std::string>("right_base_frame", "right_base");
    left_tool_frame_   = declare_parameter<std::string>("left_tool_frame",  "left_tool");
    right_tool_frame_  = declare_parameter<std::string>("right_tool_frame", "right_tool");
    servo_command_frame_ = declare_parameter<std::string>("servo_command_frame", "base"); // "base" or "tool"

    // Gains & limits
    Kp_pos_ = declare_parameter<double>("Kp_pos", 1.5);     // [1/s]
    Kp_rot_ = declare_parameter<double>("Kp_rot", 1.0);     // [1/s]
    v_max_lin_ = declare_parameter<double>("v_max_lin", 0.15); // m/s
    v_max_ang_ = declare_parameter<double>("v_max_ang", 0.8);  // rad/s

    // Stop thresholds
    eps_pos_ = declare_parameter<double>("eps_pos", 0.002); // 2 mm
    eps_ang_ = declare_parameter<double>("eps_ang", 0.02);  // ~1.1 deg

    // ===== Service Clients =====
    cli_rb3_ = this->create_client<RB3Srv>("zmk_tcppos_rb3");
    cli_ur3_ = this->create_client<UR3Srv>("zmk_tcppos_ur3e");

    cli_grip_open_  = this->create_client<std_srvs::srv::Trigger>("zmk_girp_open");
    cli_grip_close_ = this->create_client<std_srvs::srv::Trigger>("zmk_girp_close");

    hand_service_name_ = declare_parameter<std::string>("hand_service_name", "/Setangle");
    cli_hand_setangle_ = this->create_client<HandSetAngleSrv>(hand_service_name_);

    bimanual_srv_name_ = declare_parameter<std::string>("bimanual_srv_name", "zmk_move_to_tcppos");
    use_bimanual_srv_  = declare_parameter<bool>("use_bimanual_srv", true);
    client_bimanual_tcppos_ = this->create_client<dual_arm_msg::srv::Movetcppos>(bimanual_srv_name_);

    // ===== Servo Publishers & Camera Sub =====
    pub_left_twist_  = this->create_publisher<geometry_msgs::msg::TwistStamped>(left_servo_twist_topic_, 10);
    pub_right_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(right_servo_twist_topic_, 10);

    sub_cam_tcp_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      camera_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        cam_tcp_latest_ = *msg; // 최신만 유지
      });

    // ===== Sequence =====
    seq_rb3_ = {
      {-0.05367,-0.47036,0.35057,-3.10528981,-0.04380776,0.015708}, //0
      {-0.0407, -0.44568,0.450,   1.7228145,  0.17715092,1.80205245},//1
      {-0.04642,-0.44152,0.11987, 1.76208441, 0.24801129,1.77238186},//2
      { 0.050,  -0.37708,0.31162, 0.15708,    0.2003638, 1.4264576}, //3
      { 0.1483, -0.37773,0.30176, 0.1439897,  0.1534144, 1.4289011}, //4
      { 0.050,  -0.37708,0.31162, 0.15708,    0.2003638, 1.4264576}, //5
      {-0.04381,-0.45377,0.13585, 1.75492856, 0.19338248,1.8102555}, //6
      {-0.0407, -0.44568,0.31162, 1.7228145,  0.17715092,1.80205245},//7
      {-0.05367,-0.47036,0.35057,-3.10528981,-0.04380776,0.015708}, //8
    };
    seq_ur3_ = {
      {-0.140,-0.410,0.350, 0.0,-3.14, 0.0}, //0
      {-0.140,-0.410,0.165, 0.0,-3.14, 0.0}, //1
      {-0.13056,-0.32030,0.46642, 2.249,2.489,-2.523}, //2
      {-0.187,  -0.32030,0.46642, 2.249,2.489,-2.523}, //3
      {-0.13056,-0.32030,0.46642, 2.249,2.489,-2.523}, //4
      {-0.140,-0.410,0.350, 0.0,-3.14, 0.0}, //5
    };

    ur3_grip_before_[0] = GripperAction::OPEN;
    ur3_grip_after_[1]  = GripperAction::CLOSE;
    ur3_grip_after_[3]  = GripperAction::OPEN;

    rb3_grip_after_[0]  = GripperAction::OPEN;
    rb3_grip_before_[2] = GripperAction::PINCH;
    rb3_grip_after_[2]  = GripperAction::CLOSE;
    rb3_grip_before_[3] = GripperAction::CLOSE;
    rb3_grip_after_[6]  = GripperAction::PINCH;
    rb3_grip_after_[7]  = GripperAction::OPEN;

    // 동기점: 여기서 다음 스텝(4,3) 대신 Servo 세션 진입
    sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/3});

    // Start timer: 대기 → 준비 완료 → 시퀀스 시작
    start_timer_ = this->create_wall_timer(300ms, [this] {
      if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s) ||
          !cli_grip_open_->wait_for_service(0s) || !cli_grip_close_->wait_for_service(0s) ||
          !cli_hand_setangle_->wait_for_service(0s) ||
          (use_bimanual_srv_ && !client_bimanual_tcppos_->wait_for_service(0s))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[%s] Waiting for services (rb3/ur3e/gripper/hand%s). hand_service=%s, bimanual=%s",
          ts().c_str(), use_bimanual_srv_ ? "/bimanual" : "",
          hand_service_name_.c_str(), bimanual_srv_name_.c_str());
        return;
      }
      const auto right_status = resolve_status_topic(right_fjt_base_);
      const auto left_status  = resolve_status_topic(left_fjt_base_);
      if (right_status.empty() || left_status.empty()) {
        RCLCPP_ERROR(get_logger(),
          "[%s] Could not resolve action status topics. right_base=%s left_base=%s",
          ts().c_str(), right_fjt_base_.c_str(), left_fjt_base_.c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "[%s] [RESOLVED] RIGHT status: %s", ts().c_str(), right_status.c_str());
      RCLCPP_INFO(get_logger(), "[%s] [RESOLVED] LEFT  status: %s", ts().c_str(), left_status.c_str());

      sub_right_status_ = this->create_subscription<GoalStatusArray>(
        right_status, rclcpp::QoS(50),
        std::bind(&MoveSequenceClient::on_right_status, this, std::placeholders::_1));
      sub_left_status_ = this->create_subscription<GoalStatusArray>(
        left_status, rclcpp::QoS(50),
        std::bind(&MoveSequenceClient::on_left_status, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(), "[%s] Services ready. Using hand_service=%s, bimanual=%s (use=%s)",
                  ts().c_str(), hand_service_name_.c_str(), bimanual_srv_name_.c_str(),
                  use_bimanual_srv_ ? "true" : "false");

      start_timer_->cancel();
      start_sequence();
    });

    // Servo 종료용 서비스(옵션): 외부에서 호출하면 세션 종료
    srv_servo_done_ = this->create_service<std_srvs::srv::Trigger>(
      "servo_done", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> resp){
        servo_done_ = true;
        resp->success = true;
        resp->message = "servo stop requested";
      });
  }

private:
  // ===================== 원래 상태머신 필드 =====================
  struct PairSync {
    size_t rb3_idx;
    size_t ur3_idx;
    size_t rb3_next;
    size_t ur3_next;
    bool reached_rb3 = false;
    bool reached_ur3 = false;
    bool fired = false;
  };
  std::vector<PairSync> sync_points_;

  std::optional<std::pair<size_t,size_t>> sync_inflight_;
  bool rb3_inflight_done_ = false;
  bool ur3_inflight_done_ = false;

  rclcpp::Client<RB3Srv>::SharedPtr cli_rb3_;
  rclcpp::Client<UR3Srv>::SharedPtr cli_ur3_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_open_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_grip_close_;
  rclcpp::Client<HandSetAngleSrv>::SharedPtr cli_hand_setangle_;
  std::string hand_service_name_;
  rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedPtr client_bimanual_tcppos_;
  std::string bimanual_srv_name_;
  bool use_bimanual_srv_ = true;

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

  std::chrono::steady_clock::time_point t0_steady_;
  std::chrono::steady_clock::time_point t_send_rb3_, t_send_ur3_;
  std::chrono::steady_clock::time_point t_active_rb3_, t_active_ur3_;

  bool exit_when_done_{true};
  bool shutdown_scheduled_{false};
  rclcpp::TimerBase::SharedPtr shutdown_timer_;

  // ===================== Servo / Camera / TF 필드 =====================
  // MoveIt Servo
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_left_twist_, pub_right_twist_;
  std::string left_servo_twist_topic_, right_servo_twist_topic_;

  // Camera target
  std::string camera_pose_topic_;
  std::string camera_frame_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cam_tcp_;
  std::optional<geometry_msgs::msg::PoseStamped> cam_tcp_latest_;

  // Frames
  std::string left_base_frame_, right_base_frame_, left_tool_frame_, right_tool_frame_;
  std::string servo_command_frame_; // "base" or "tool"

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Gains & limits
  double Kp_pos_{1.5}, Kp_rot_{1.0};
  double v_max_lin_{0.15}, v_max_ang_{0.8};
  double eps_pos_{0.002}, eps_ang_{0.02};

  // Servo session state
  bool servo_session_{false};
  std::optional<std::pair<size_t,size_t>> resume_after_servo_; // {rb3_next, ur3_next}
  rclcpp::TimerBase::SharedPtr servo_timer_;
  std::atomic<bool> servo_deadman_{true};
  std::atomic<bool> servo_done_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_servo_done_;

  // ===================== 유틸 =====================
  std::string ts() const {
    using namespace std::chrono;
    auto now = steady_clock::now();
    auto ms  = duration_cast<milliseconds>(now - t0_steady_).count();
    std::ostringstream oss; oss << std::setw(9) << ms << "ms";
    return oss.str();
  }
  static bool ends_with(const std::string &s, const std::string &suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
  }
  static bool any_active(const GoalStatusArray &arr) {
    for (const auto &st : arr.status_list) {
      if (st.status == GoalStatus::STATUS_ACCEPTED ||
          st.status == GoalStatus::STATUS_EXECUTING ||
          st.status == GoalStatus::STATUS_CANCELING) return true;
    }
    return false;
  }

  std::string resolve_status_topic(const std::string &base)
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

  // ===================== 시퀀스 시작 =====================
  void start_sequence() {
    RCLCPP_INFO(this->get_logger(), "[%s] === Sequence PREP ===", ts().c_str());
    prepare_and_fire_first_pair();
  }

  void prepare_and_fire_first_pair() {
    const bool need_rb3 = rb3_grip_before_.count(0) && rb3_grip_before_[0] != GripperAction::NONE;
    const bool need_ur3 = ur3_grip_before_.count(0) && ur3_grip_before_[0] != GripperAction::NONE;

    if (!need_rb3 && !need_ur3) {
      RCLCPP_INFO(this->get_logger(), "[%s] No BEFORE hooks at idx0 → FIRE BOTH step0 now", ts().c_str());
      sync_inflight_ = std::make_pair(0u, 0u);
      rb3_inflight_done_ = ur3_inflight_done_ = false;
      fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
      return;
    }

    auto pending = std::make_shared<std::atomic<int>>(0);
    if (need_rb3) pending->fetch_add(1);
    if (need_ur3) pending->fetch_add(1);

    auto after = [this, pending]() {
      if (pending->fetch_sub(1) == 1) {
        RCLCPP_INFO(this->get_logger(), "[%s] PREP hooks done → FIRE BOTH step0", ts().c_str());
        sync_inflight_ = std::make_pair(0u, 0u);
        rb3_inflight_done_ = ur3_inflight_done_ = false;
        fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
      }
    };

    if (need_rb3) request_rb_hand(rb3_grip_before_[0], after);
    if (need_ur3) request_ur_gripper(ur3_grip_before_[0], after);
  }

  void fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks)
  {
    if (!apply_before_hooks) {
      sync_fire_timer_ = this->create_wall_timer(
        0ms,
        [this, rb3_idx, ur3_idx](){
          RCLCPP_INFO(this->get_logger(),
            "[%s] FIRE BOTH NOW (RB3=%zu, UR3e=%zu) — same tick",
            ts().c_str(), rb3_idx, ur3_idx);

          if (use_bimanual_srv_) {
            const auto &r = seq_rb3_[rb3_idx];
            const auto &l = seq_ur3_[ur3_idx];
            call_zmk_move_to_bimanual_tcppos_async(
              /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
              /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
          } else {
            send_ur3_step(ur3_idx);
            send_rb3_step(rb3_idx);
          }
          sync_fire_timer_->cancel();
        }
      );
      return;
    }

    // (hooks 처리 버전: 생략 가능 – 필요시 기존 코드 유지)
    send_rb3_step(rb3_idx);
    send_ur3_step(ur3_idx);
  }

  // ===================== BIMANUAL CALLS =====================
  void call_zmk_move_to_bimanual_tcppos_async(
      double l_x, double l_y, double l_z, double l_rx, double l_ry, double l_rz,
      double r_x, double r_y, double r_z, double r_rx, double r_ry, double r_rz)
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
      "[%s] [BIMANUAL] SEND tcppos (UR3e & RB3) via %s", ts().c_str(), bimanual_srv_name_.c_str());

    client_bimanual_tcppos_->async_send_request(
      req,
      [this](rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedFuture f){
        bool ok = false; std::string msg = "no response";
        try { auto resp = f.get(); ok = resp && resp->success; if (resp) msg = resp->message; }
        catch (const std::exception& e) { msg = e.what(); }
        catch (...) {}
        RCLCPP_INFO(this->get_logger(), "[%s] [BIMANUAL] request SENT %s (%s)",
                    ts().c_str(), ok ? "OK" : "FAILED", msg.c_str());
      });
  }

  // ===================== RB3 / UR3 Steps =====================
  void start_rb3_step(size_t i)
  {
    if (i >= seq_rb3_.size()) { if (!rb3_done_all_) { rb3_done_all_ = true; maybe_finish(); } return; }
    if (rb3_grip_before_.count(i) && rb3_grip_before_[i] != GripperAction::NONE) {
      request_rb_hand(rb3_grip_before_[i], /*on_done=*/nullptr);
      send_rb3_step(i);
    } else {
      send_rb3_step(i);
    }
  }
  void send_rb3_step(size_t i)
  {
    if (i >= seq_rb3_.size()) return;
    const auto &p = seq_rb3_[i];
    auto req = std::make_shared<RB3Srv::Request>();
    req->right_x = p.x; req->right_y = p.y; req->right_z = p.z;
    req->right_rx = p.rx; req->right_ry = p.ry; req->right_rz = p.rz;

    rb3_waiting_ = true; rb3_seen_active_ = false; t_send_rb3_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "[%s] [RB3] SEND step %zu", ts().c_str(), i);
    cli_rb3_->async_send_request(
      req,
      [this, i](rclcpp::Client<RB3Srv>::SharedFuture future) {
        auto resp = future.get();
        if (resp && resp->success) {
          RCLCPP_INFO(this->get_logger(), "[%s] [RB3] step %zu SENT OK: %s",
                      ts().c_str(), i, resp->message.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "[%s] [RB3] step %zu SEND FAILED", ts().c_str(), i);
        }
      });
  }
  void advance_rb3_after_done(size_t just_finished_idx)
  {
    if (rb3_grip_after_.count(just_finished_idx) &&
        rb3_grip_after_[just_finished_idx] != GripperAction::NONE) {
      request_rb_hand(rb3_grip_after_[just_finished_idx], [this]{ ++idx_rb3_; start_rb3_step(idx_rb3_); });
    } else {
      ++idx_rb3_;
      start_rb3_step(idx_rb3_);
    }
  }

  void start_ur3_step(size_t i)
  {
    if (i >= seq_ur3_.size()) { if (!ur3_done_all_) { ur3_done_all_ = true; maybe_finish(); } return; }
    if (ur3_grip_before_.count(i) && ur3_grip_before_[i] != GripperAction::NONE) {
      request_ur_gripper(ur3_grip_before_[i], [this, i]{ send_ur3_step(i); });
    } else {
      send_ur3_step(i);
    }
  }
  void send_ur3_step(size_t i)
  {
    if (i >= seq_ur3_.size()) return;
    const auto &p = seq_ur3_[i];
    auto req = std::make_shared<UR3Srv::Request>();
    req->left_x = p.x; req->left_y = p.y; req->left_z = p.z;
    req->left_rx = p.rx; req->left_ry = p.ry; req->left_rz = p.rz;

    ur3_waiting_ = true; ur3_seen_active_ = false; t_send_ur3_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] SEND step %zu", ts().c_str(), i);
    cli_ur3_->async_send_request(
      req,
      [this, i](rclcpp::Client<UR3Srv>::SharedFuture future) {
        auto resp = future.get();
        if (resp && resp->success) {
          RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] step %zu SENT OK: %s",
                      ts().c_str(), i, resp->message.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "[%s] [UR3e] step %zu SEND FAILED", ts().c_str(), i);
        }
      });
  }
  void advance_ur3_after_done(size_t just_finished_idx)
  {
    if (ur3_grip_after_.count(just_finished_idx) &&
        ur3_grip_after_[just_finished_idx] != GripperAction::NONE) {
      request_ur_gripper(ur3_grip_after_[just_finished_idx], [this]{ ++idx_ur3_; start_ur3_step(idx_ur3_); });
    } else {
      ++idx_ur3_;
      start_ur3_step(idx_ur3_);
    }
  }

  // ===================== 상태 콜백 =====================
  void on_right_status(GoalStatusArray::SharedPtr msg)
  {
    const bool active = any_active(*msg);
    if (rb3_waiting_) {
      if (active && !rb3_seen_active_) {
        rb3_seen_active_ = true;
        t_active_rb3_ = std::chrono::steady_clock::now();
      }
      if (rb3_seen_active_ && !active) {
        rb3_waiting_ = false;
        RCLCPP_INFO(this->get_logger(), "[%s] [RB3] DONE at idx=%zu", ts().c_str(), idx_rb3_);

        if (sync_inflight_.has_value() && idx_rb3_ == sync_inflight_->first) {
          run_after_hook_barrier_only(/*is_left=*/false, idx_rb3_);
          rb3_inflight_done_ = true;
          check_inflight_and_advance_after_both_done();
          return;
        }
        if (handle_sync_reached(/*is_left=*/false, idx_rb3_)) return;

        advance_rb3_after_done(idx_rb3_);
      }
    }
  }
  void on_left_status(GoalStatusArray::SharedPtr msg)
  {
    const bool active = any_active(*msg);
    if (ur3_waiting_) {
      if (active && !ur3_seen_active_) {
        ur3_seen_active_ = true;
        t_active_ur3_ = std::chrono::steady_clock::now();
      }
      if (ur3_seen_active_ && !active) {
        ur3_waiting_ = false;
        RCLCPP_INFO(this->get_logger(), "[%s] [UR3e] DONE at idx=%zu", ts().c_str(), idx_ur3_);

        if (sync_inflight_.has_value() && idx_ur3_ == sync_inflight_->second) {
          run_after_hook_barrier_only(/*is_left=*/true, idx_ur3_);
          ur3_inflight_done_ = true;
          check_inflight_and_advance_after_both_done();
          return;
        }
        if (handle_sync_reached(/*is_left=*/true, idx_ur3_)) return;

        advance_ur3_after_done(idx_ur3_);
      }
    }
  }

  // ===================== 동기점 처리: 여기서 Servo 진입 =====================
  bool handle_sync_reached(bool is_left, size_t just_finished_idx)
  {
    bool matched_any = false;
    for (auto &sp : sync_points_) {
      if (sp.fired) continue;

      if (!is_left && just_finished_idx == sp.rb3_idx) {
        sp.reached_rb3 = true; matched_any = true;
        run_after_hook_barrier_only(/*is_left=*/false, just_finished_idx);
      }
      if ( is_left && just_finished_idx == sp.ur3_idx) {
        sp.reached_ur3 = true; matched_any = true;
        run_after_hook_barrier_only(/*is_left=*/true, just_finished_idx);
      }

      if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
        sp.fired = true;

        // 원래는 idx_rb3_=4, idx_ur3_=3에서 발사
        // → 대신 서보 세션 진입
        resume_after_servo_ = std::make_pair(sp.rb3_next, sp.ur3_next); // {4,3}
        enter_servo_session_();
        return true;
      }
    }
    return matched_any;
  }

  void check_inflight_and_advance_after_both_done()
  {
    if (!sync_inflight_.has_value()) return;
    if (!(rb3_inflight_done_ && ur3_inflight_done_)) return;

    auto [rb3_idx, ur3_idx] = *sync_inflight_;
    sync_inflight_.reset();
    rb3_inflight_done_ = ur3_inflight_done_ = false;

    ++idx_rb3_;  start_rb3_step(idx_rb3_);
    ++idx_ur3_;  start_ur3_step(idx_ur3_);
  }

  void run_after_hook_barrier_only(bool is_left, size_t finished_idx)
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
      if (is_left) request_ur_gripper(act, /*on_done=*/nullptr);
      else         request_rb_hand(act,    /*on_done=*/nullptr);
    }
  }

  void maybe_finish() {
    if (rb3_done_all_ && ur3_done_all_) {
      RCLCPP_INFO(this->get_logger(), "[%s] === Sequence ALL DONE ===", ts().c_str());
      if (exit_when_done_ && !shutdown_scheduled_) {
        shutdown_scheduled_ = true;
        shutdown_timer_ = this->create_wall_timer(200ms, [this]() {
          RCLCPP_INFO(this->get_logger(), "[%s] Shutting down (exit_when_done=true).", ts().c_str());
          rclcpp::shutdown();
        });
      }
    }
  }

  // ===================== Servo 세션 진입/루프/종료 =====================
  void enter_servo_session_()
  {
    if (servo_session_) return;
    servo_session_ = true;
    servo_deadman_ = true;
    servo_done_ = false;

    RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] Enter SERVO session (visual servo)", ts().c_str());

    // 100 Hz 루프
    servo_timer_ = this->create_wall_timer(10ms, [this](){
      if (!servo_session_) { servo_timer_->cancel(); return; }
      if (!servo_deadman_) { publish_zero_twist_(); end_servo_and_resume_(); return; }
      if (!cam_tcp_latest_.has_value()) return; // 아직 타깃 없음

      // 왼/오른 팔 모두 동일 타깃을 쫓는다고 가정(필요 시 좌/우 별도의 타깃 구독)
      // 1) 타깃 포즈를 각 팔 base 프레임으로 변환
      geometry_msgs::msg::PoseStamped target_in_left_base;
      geometry_msgs::msg::PoseStamped target_in_right_base;
      try {
        target_in_left_base  = tf_buffer_.transform(*cam_tcp_latest_, left_base_frame_, 50ms);
        target_in_right_base = tf_buffer_.transform(*cam_tcp_latest_, right_base_frame_, 50ms);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF transform failed: %s", ex.what());
        return;
      }

      // 2) 현재 EE 포즈를 각 base에서 조회 (TF)
      geometry_msgs::msg::TransformStamped T_left_base_tool, T_right_base_tool;
      try {
        T_left_base_tool  = tf_buffer_.lookupTransform(left_base_frame_,  left_tool_frame_,  rclcpp::Time(0), 50ms);
        T_right_base_tool = tf_buffer_.lookupTransform(right_base_frame_, right_tool_frame_, rclcpp::Time(0), 50ms);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
        return;
      }

      // 3) 오차 계산 (base frame 기준)
      auto [vL, wL, pos_err_L, ang_err_L] = compute_twist_from_error_(
        T_left_base_tool, target_in_left_base.pose, left_base_frame_);
      auto [vR, wR, pos_err_R, ang_err_R] = compute_twist_from_error_(
        T_right_base_tool, target_in_right_base.pose, right_base_frame_);

      // 4) 종료 조건(둘 다 충분히 근접)
      if (pos_err_L < eps_pos_ && ang_err_L < eps_ang_ &&
          pos_err_R < eps_pos_ && ang_err_R < eps_ang_) {
        servo_done_ = true;
      }

      // 5) 퍼블리시 (Servo 파라미터의 command_frame에 맞춰 header.frame_id 설정)
      geometry_msgs::msg::TwistStamped tL, tR;
      tL.header.stamp = now();
      tR.header.stamp = now();
      if (servo_command_frame_ == "tool") {
        // base에서 계산된 twist를 tool frame으로 회전만 변환(선속/각속 모두)
        rotate_twist_to_tool_(T_left_base_tool,  vL, wL);
        rotate_twist_to_tool_(T_right_base_tool, vR, wR);
        tL.header.frame_id = left_tool_frame_;
        tR.header.frame_id = right_tool_frame_;
      } else {
        tL.header.frame_id = left_base_frame_;
        tR.header.frame_id = right_base_frame_;
      }
      tL.twist.linear.x  = vL[0]; tL.twist.linear.y  = vL[1]; tL.twist.linear.z  = vL[2];
      tL.twist.angular.x = wL[0]; tL.twist.angular.y = wL[1]; tL.twist.angular.z = wL[2];
      tR.twist.linear.x  = vR[0]; tR.twist.linear.y  = vR[1]; tR.twist.linear.z  = vR[2];
      tR.twist.angular.x = wR[0]; tR.twist.angular.y = wR[1]; tR.twist.angular.z = wR[2];

      pub_left_twist_->publish(tL);
      pub_right_twist_->publish(tR);

      if (servo_done_) { publish_zero_twist_(); end_servo_and_resume_(); }
    });
  }

  // base frame에서 EE(current) → target 오차로부터 twist 산출
  // 반환: (v[3], w[3], pos_err_norm, ang_err_abs)
  std::tuple<std::array<double,3>, std::array<double,3>, double, double>
  compute_twist_from_error_(const geometry_msgs::msg::TransformStamped& T_base_tool,
                            const geometry_msgs::msg::Pose& target_in_base,
                            const std::string& /*base_frame*/)
  {
    // 현재 EE in base
    const auto& pC = T_base_tool.transform.translation;
    const auto& qC = T_base_tool.transform.rotation;

    // 타깃 EE in base
    const auto& pT = target_in_base.position;
    const auto& qT = target_in_base.orientation;

    // position error (base)
    std::array<double,3> eP{
      (double)(pT.x - pC.x),
      (double)(pT.y - pC.y),
      (double)(pT.z - pC.z)
    };
    double pos_err = std::sqrt(eP[0]*eP[0] + eP[1]*eP[1] + eP[2]*eP[2]);

    // orientation error using quaternion: q_err = qC^-1 * qT  (current→target)
    tf2::Quaternion qC_tf(qC.x, qC.y, qC.z, qC.w);
    tf2::Quaternion qT_tf(qT.x, qT.y, qT.z, qT.w);
    tf2::Quaternion qErr = qC_tf.inverse() * qT_tf;
    qErr.normalize();

    // 소각 근사: axis * angle
    double ang = 2.0 * std::atan2(std::sqrt(qErr.x()*qErr.x()+qErr.y()*qErr.y()+qErr.z()*qErr.z()), qErr.w());
    tf2::Vector3 axis(0,0,0);
    if (std::abs(ang) > 1e-6) {
      axis = tf2::Vector3(qErr.x(), qErr.y(), qErr.z());
      axis.normalize();
    }
    std::array<double,3> eW{ axis.x()*ang, axis.y()*ang, axis.z()*ang };
    double ang_err = std::abs(ang);

    // P 제어 → 속도 참조
    std::array<double,3> v{ Kp_pos_*eP[0], Kp_pos_*eP[1], Kp_pos_*eP[2] };
    std::array<double,3> w{ Kp_rot_*eW[0], Kp_rot_*eW[1], Kp_rot_*eW[2] };
    clamp_twist_(v, w);
    return {v, w, pos_err, ang_err};
  }

  void clamp_twist_(std::array<double,3>& v, std::array<double,3>& w)
  {
    auto clamp3 = [](std::array<double,3>& a, double maxnorm){
      double n = std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
      if (n > maxnorm && n > 1e-9) { double s = maxnorm / n; a[0]*=s; a[1]*=s; a[2]*=s; }
    };
    clamp3(v, v_max_lin_);
    clamp3(w, v_max_ang_);
  }

  // base→tool 회전으로 twist를 tool 프레임으로 회전(선속/각속 동일하게 R^T 곱)
  void rotate_twist_to_tool_(const geometry_msgs::msg::TransformStamped& T_base_tool,
                             std::array<double,3>& v, std::array<double,3>& w)
  {
    const auto &q = T_base_tool.transform.rotation;
    tf2::Quaternion qbt(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 Rbt(qbt); // base->tool
    tf2::Matrix3x3 Rtb = Rbt.transpose();

    auto rot3 = [&](const std::array<double,3>& x)->std::array<double,3>{
      tf2::Vector3 v(x[0], x[1], x[2]);
      tf2::Vector3 y = Rtb * v; // base vec를 tool로
      return {y.x(), y.y(), y.z()};
    };
    v = rot3(v);
    w = rot3(w);
  }

  void publish_zero_twist_()
  {
    geometry_msgs::msg::TwistStamped z;
    z.header.stamp = now();
    z.header.frame_id = (servo_command_frame_=="tool") ? left_tool_frame_ : left_base_frame_;
    pub_left_twist_->publish(z);
    z.header.frame_id = (servo_command_frame_=="tool") ? right_tool_frame_ : right_base_frame_;
    pub_right_twist_->publish(z);
  }

  void end_servo_and_resume_()
  {
    if (!servo_session_) return;
    servo_session_ = false;

    if (resume_after_servo_.has_value()) {
      idx_rb3_ = resume_after_servo_->first;   // 4
      idx_ur3_ = resume_after_servo_->second;  // 3
      resume_after_servo_.reset();
      RCLCPP_INFO(this->get_logger(), "[%s] [SERVO→RESUME] RB3=%zu, UR3=%zu",
                  ts().c_str(), idx_rb3_, idx_ur3_);
      // 원래 로직 재개(양팔 동시 발사)
      fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
    } else {
      ++idx_rb3_; start_rb3_step(idx_rb3_);
      ++idx_ur3_; start_ur3_step(idx_ur3_);
    }
  }

  // ===================== 그리퍼/핸드 =====================
  void request_ur_gripper(GripperAction action, std::function<void()> on_done)
  {
    if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto cli = (action == GripperAction::CLOSE) ? cli_grip_close_ : cli_grip_open_;
    if (!cli->wait_for_service(0s)) {
      if (on_done) on_done();
      return;
    }
    auto t0 = std::chrono::steady_clock::now();
    cli->async_send_request(
      req,
      [this, action, on_done, t0](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        bool ok = false; try { auto resp = future.get(); ok = resp && resp->success; } catch (...) {}
        (void)ok; (void)t0;
        if (on_done) on_done();
      });
  }

  void request_rb_hand(GripperAction action, std::function<void()> on_done)
  {
    if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
    if (!cli_hand_setangle_->wait_for_service(0s)) {
      if (on_done) on_done();
      return;
    }

    auto req = std::make_shared<HandSetAngleSrv::Request>();
    // Preset
    if (action == GripperAction::CLOSE) {
      req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 645;
      req->angle3 = 671;  req->angle4 = 260;  req->angle5 = 0;
    } else if(action == GripperAction::PINCH) {
      req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
      req->angle3 = 1000; req->angle4 = 400;  req->angle5 = 0;
    } else { // OPEN
      req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
      req->angle3 = 1000; req->angle4 = 1000; req->angle5 = 0;
    }
    req->hand_id = 1;
    req->status  = "set_angle";

    cli_hand_setangle_->async_send_request(
      req,
      [this, on_done](rclcpp::Client<HandSetAngleSrv>::SharedFuture future) {
        bool ok = false; try { auto resp = future.get(); ok = (resp != nullptr); } catch (...) {}
        (void)ok;
        if (on_done) on_done();
      });
  }
};

// ===================== main =====================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveSequenceClient>());
  rclcpp::shutdown();
  return 0;
}
