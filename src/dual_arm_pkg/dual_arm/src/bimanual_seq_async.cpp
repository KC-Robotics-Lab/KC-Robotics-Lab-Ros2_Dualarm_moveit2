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
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "dual_arm_msg/srv/rotblock.hpp"
#include "dual_arm_msg/srv/movetcppos_rb3.hpp"
#include "dual_arm_msg/srv/movetcppos_ur3.hpp"
#include "dual_arm_msg/srv/objpickpoint.hpp"
#include "dual_arm_msg/srv/movetcppos.hpp"
#include "inspire_hand_interface/srv/setangle.hpp"

using namespace std::chrono_literals;
using action_msgs::msg::GoalStatusArray;
using action_msgs::msg::GoalStatus;

using RB3Srv = dual_arm_msg::srv::MovetcpposRB3;
using UR3Srv = dual_arm_msg::srv::MovetcpposUR3;
using ObjPickSrv = dual_arm_msg::srv::Objpickpoint;
using RotblockSrv = dual_arm_msg::srv::Rotblock;
using HandSetAngleSrv = inspire_hand_interface::srv::Setangle;

struct Pose6 { double x, y, z, rx, ry, rz; };
enum class GripperAction { NONE = 0, OPEN, CLOSE, PINCH, HOME, RESTART};

// ===== DEFAULT POSES (startup home) =====
static const Pose6 RB3_DEFAULT_POSE = {
  -0.00494, -0.36764,  0.14923, -3.10528981, -0.04380776,  0.015708
};

static const Pose6 UR3_DEFAULT_POSE = {
  -0.18941, -0.25945, 0.30880, 0.264, -2.675, 0.349
  // -0.17260, -0.22924,  0.48026,  1.013,  3.069, -0.424
  // -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 
};

#define LOGP(fmt, ...) RCLCPP_INFO(get_logger(), "[%s] " fmt, ts().c_str(), ##__VA_ARGS__)
#define LOGE(fmt, ...) RCLCPP_ERROR(get_logger(), "[%s] " fmt, ts().c_str(), ##__VA_ARGS__)

class MoveSequenceClient : public rclcpp::Node
{
public:
  MoveSequenceClient();

private:
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

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_pickblock_;    // was Trigger
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_degcheck_;     // was Trigger
  rclcpp::Service<RotblockSrv>::SharedPtr srv_rotblock_;

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

  // --- 멀티 훅: AFTER (기존)
  std::unordered_map<size_t, std::vector<GripperAction>> rb3_grip_after_multi_;
  // --- 멀티 훅: BEFORE (신규)
  std::unordered_map<size_t, std::vector<GripperAction>> rb3_grip_before_multi_;

  rclcpp::TimerBase::SharedPtr sync_fire_timer_;

  bool hold_until_degcheck_{false};
  bool hold_until_rotblock_{false};

  std::chrono::steady_clock::time_point t0_steady_;
  std::chrono::steady_clock::time_point t_send_rb3_, t_send_ur3_;
  std::chrono::steady_clock::time_point t_active_rb3_, t_active_ur3_;

  bool exit_when_done_{true};
  bool shutdown_scheduled_{false};
  rclcpp::TimerBase::SharedPtr shutdown_timer_;

  // base snapshot (main)
  std::vector<Pose6> base_seq_rb3_, base_seq_ur3_;
  std::unordered_map<size_t, GripperAction> base_ur3_grip_before_;
  std::unordered_map<size_t, GripperAction> base_ur3_grip_after_;
  std::unordered_map<size_t, GripperAction> base_rb3_grip_before_;
  std::unordered_map<size_t, GripperAction> base_rb3_grip_after_;
  std::vector<PairSync> base_sync_points_;
  std::unordered_map<size_t, std::vector<GripperAction>> base_rb3_grip_after_multi_;
  std::unordered_map<size_t, std::vector<GripperAction>> base_rb3_grip_before_multi_;

  // soft-restart flags
  std::atomic<bool> restart_pending_{false};
  bool block_progress_{false};
  rclcpp::TimerBase::SharedPtr restart_watchdog_;

  rclcpp::TimerBase::SharedPtr apply_wait_timer_;
  rclcpp::TimerBase::SharedPtr restart_delay_timer_;
  bool auto_restart_once_ = false;

  // ---------- TF ----------
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string rb3_base_frame_;
  std::string rb3_tcp_frame_;

  // ---- 직렬 훅 실행(중복 포함)용 상태 ----
  bool rb3_before_already_done_for_next_ = false;               // 다음 스텝의 before를 이미 실행했음
  rclcpp::TimerBase::SharedPtr rb3_hook_cooldown_timer_;        // 동일 연속 명령 쿨다운
  std::optional<GripperAction> rb3_last_hook_sent_;             // 직전에 보낸 훅 기록

  // ---------- rotblock 전용 완료 감지 플래그/타임스탬프/설명 ----------
  bool rotblock_waiting_{false};
  bool rotblock_seen_active_{false};
  std::chrono::steady_clock::time_point t_send_rotblock_, t_active_rotblock_;
  std::string last_rotblock_desc_;

  // --- ROTBLOCK 완료 알림(서비스 클라이언트) ---
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr rotblock_done_cli_; // was Trigger
  std::string rotblock_done_srv_name_;

  // ROTBLOCK 완료 1회 알림 가드
  bool rotblock_done_notified_ = false;

  // --- 메인 시퀀스 완료 알림(서비스 클라이언트) ---
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr mainseq_done_cli_;  // was Trigger
  std::string mainseq_done_srv_name_;
  bool mainseq_done_notified_ = false;

  // ★★★ 4/5 step 완료 알림(서비스 클라이언트) 추가 ★★★
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr step4_done_cli_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr step5_done_cli_;
  std::string step4_done_srv_name_;
  std::string step5_done_srv_name_;

  bool step4_done_notified_{false};

  std::mutex rotblock_mutex_;
  std::condition_variable rotblock_cv_;
  bool rotblock_done_flag_ = false;
  std::string rotblock_result_msg_;

  // ===== objpickpoint 저장용(총 12개) =====
  rclcpp::Service<ObjPickSrv>::SharedPtr srv_objpickpoint_;

  mutable std::mutex objpick_mtx_;
  Pose6 objpick_rb3_{};
  Pose6 objpick_ur3_{};
  std::atomic<bool> objpick_ready_{false};   // 12개 모두 업데이트 되었는지

  // 메인시퀀스 시작 트리거 서버
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_start_mainseq_;// was Trigger
  // 시스템 준비/실행 상태
  std::atomic<bool> system_ready_{false};
  std::atomic<bool> mainseq_running_{false};

  // 자동 시작 옵션 (파라미터)
  bool auto_start_{false};

  // 메인시퀀스 시작 전엔 서비스 호출 막기
  std::atomic<bool> services_unlocked_{false};

  inline bool services_blocked() const noexcept {
    return !services_unlocked_.load(std::memory_order_acquire);
  }

  // ---------- helpers (decl) ----------
  std::string ts() const;
  void move_to_default_pose_on_start();
  static bool ends_with(const std::string &s, const std::string &suffix);
  static bool any_active(const GoalStatusArray &arr);
  std::string resolve_status_topic(const std::string &base);

  std::string controller_ns_from_fjt_base(const std::string &fjt_base);
  void configure_soft_stop(double seconds);
  void notify_rotblock_done(const std::string &msg);
  void notify_mainseq_done(const std::string &msg);

  void maybe_notify_step4();

  // ★★★ 4/5 step 완료 알림 helper 선언 ★★★
  void notify_4step_done(const std::string &msg);
  void notify_5step_done(const std::string &msg);

  void start_sequence();
  void reset_sequence_state();
  bool cancel_all_goals();
  bool cancel_right_only();

  void snapshot_base_sequences_hooks_sync();
  void restore_base_sequences_hooks_sync();

  void sync_reconcile_with_current_indices();

  void load_sequence_HOME();

  void prepare_and_fire_first_pair();
  void fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks);
  void fire_both_immediate(size_t rb3_idx, size_t ur3_idx);
  void fire_rb3_only_now(size_t rb3_idx);

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

  void request_soft_restart();
  void maybe_restart_after_idle();
  void do_restart_now();

  static inline double deg2rad(double d) { return d * M_PI / 180.0; }
  static inline double rad2deg(double r) { return r * 180.0 / M_PI; }
  void send_rb3_pose(const Pose6 &p);

  // ---- 직렬 RB3 훅 실행 유틸 ----
  void run_rb3_hooks_then(std::vector<GripperAction> seq, std::function<void()> then);

  // ---- RB3 BEFORE 멀티 시퀀스 구성 ----
  std::vector<GripperAction> rb3_before_seq(size_t i) const;

  Pose6 resolve_rb3_pose(size_t i) const;
  Pose6 resolve_ur3_pose(size_t i) const;

  static inline void rpy_to_rotvec(double roll, double pitch, double yaw,
                                 double &rx, double &ry, double &rz)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    const double angle = q.getAngle();      // [0, pi]
    tf2::Vector3 axis = q.getAxis();        // unit axis (angle==0이면 axis 의미 없음)

    if (angle < 1e-12) {
      rx = ry = rz = 0.0;
      return;
    }

    const tf2::Vector3 rv = axis * angle;   // rotvec = axis * angle
    rx = rv.x();
    ry = rv.y();
    rz = rv.z();
  }
};

// ===================== IMPLEMENTATION =====================

MoveSequenceClient::MoveSequenceClient()
: Node("move_sequence_client_event_driven")
, t0_steady_(std::chrono::steady_clock::now())
{
  exit_when_done_ = this->declare_parameter<bool>("exit_when_done", false);

  right_fjt_base_ = declare_parameter<std::string>(
    "right_fjt_base", "/joint_trajectory_controller/follow_joint_trajectory");
  left_fjt_base_ = declare_parameter<std::string>(
    "left_fjt_base", "/scaled_joint_trajectory_controller/follow_joint_trajectory");

  rb3_base_frame_ = declare_parameter<std::string>("rb3_base_frame", "link0");
  rb3_tcp_frame_  = declare_parameter<std::string>("rb3_tcp_frame",  "tcp");

  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

  rotblock_done_srv_name_ = declare_parameter<std::string>(
    "rotblock_done_notify_service", "/zmk_rotblock_done");
  rotblock_done_cli_ = this->create_client<std_srvs::srv::Empty>(rotblock_done_srv_name_);

  mainseq_done_srv_name_ = declare_parameter<std::string>(
    "mainseq_done_notify_service", "/finish_mainseq");
  mainseq_done_cli_  = this->create_client<std_srvs::srv::Empty>(mainseq_done_srv_name_);

  // ★★★ 4/5 step done notify 서비스 클라이언트 생성 ★★★
  step4_done_srv_name_ = declare_parameter<std::string>(
    "step4_done_notify_service", "/cam_check_prev");
  step4_done_cli_ = this->create_client<std_srvs::srv::Empty>(step4_done_srv_name_);

  step5_done_srv_name_ = declare_parameter<std::string>(
    "step5_done_notify_service", "/cam_check");
  step5_done_cli_ = this->create_client<std_srvs::srv::Empty>(step5_done_srv_name_);

  auto_start_ = this->declare_parameter<bool>("auto_start", false);
  
  // 소프트스톱 설정은 조용히 진행
  configure_soft_stop(0.5);

  // ---- main sequences/hooks/sync ----
  // seq_rb3_ = {
  //   {-0.00494, -0.36764,  0.14923, -3.10528981, -0.04380776,  0.015708},   // (0)   // Home
  //   { 0.13947, -0.38347,  0.31996,  1.7228145,   0.17715092,  1.80205245}, // (1)   // pick 위
  //   { 0.13258, -0.39074, -0.01928,  1.76208441,  0.24801129,  1.77238186}, // (2)   // pick 위치
  //   { 0.18626, -0.31841,  0.48743,  0.15708,     0.2003638,   1.4264576},  // (3)   // 카메라앞 전
  //   { 0.27916, -0.09773,  0.43470,  1.84917634, -1.4379768,   1.4158111},  // (4)   // 카메라앞
  //   { 0.18652, -0.32678,  0.47979,  0.1712168,   0.1609194,   1.3144075},  // (5)   // 결합 전
  //   { 0.27803, -0.32678,  0.47979,  0.1712168,   0.1609194,   1.3144075},  // (6)   // 결합
  //   { 0.18652, -0.27738,  0.48838,  0.15708,     0.2003638,   1.4264576},  // (7)   // 결합 전
  //   { 0.13294, -0.39570, -0.00043,  1.8107791,   0.2555162,   1.85022354}, // (8)   // place 위치
  //   { 0.13947, -0.38347,  0.31996,  1.7228145,   0.17715092,  1.80205245}, // (9)   // place 위
  //   {-0.00494, -0.36764,  0.14923, -3.10528981, -0.04380776,  0.015708},   // (10)  // Home
  // };
  // seq_ur3_ = {
  //   { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)  // Home
  //   { -0.140,   -0.410,   0.180,   0.0,   -3.14,  0.0 },      // (1)  // Pick 위치
  //   { -0.0800,  -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (2)  // 결합 전전 위치
  //   { -0.13056, -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (3)  // 결합 전 위치
  //   { -0.175,   -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (4)  // 결합
  //   { -0.13056, -0.32030, 0.46642, 2.151,  0.109, -2.257 },   // (5)  // 결합 전 위치
  //   { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (6)  // Home
  // };

  seq_rb3_ = {
    RB3_DEFAULT_POSE,
    // {-0.00494, -0.36764,  0.14923, -3.10528981, -0.04380776,  0.015708},   // (0)
    { 0.13947, -0.38347,  0.31996,  1.7228145,   0.17715092,  1.80205245}, // (1)
    { 0.13258, -0.39674, -0.01928,  1.76208441,  0.24801129,  1.77238186}, // (2)
    { 0.18626, -0.31841,  0.48743,  0.15708,     0.2003638,   1.4264576},  // (3)
    { 0.28232, -0.1337,  0.50071,  1.4826572,  -0.69010319,   1.5995943},  // (4) // 카메라앞
    { 0.18652, -0.27738,  0.48838,  0.15708,     0.2003638,   1.4264576},  // (5)
    { 0.27703, -0.33278,  0.47979,  0.1712168,   0.1609194,   1.3144075},  // (6)
    { 0.18652, -0.27738,  0.48838,  0.15708,     0.2003638,   1.4264576},  // (7)
    { 0.13255, -0.39435, -0.00131,  1.75492856,  0.19338248,  1.8102555},  // (8)
    { 0.13947, -0.38347,  0.31996,  1.7228145,   0.17715092,  1.80205245}, // (9)
    // {-0.00494, -0.36764,  0.14923, -3.10528981, -0.04380776,  0.015708},   // (10)
    RB3_DEFAULT_POSE,
  };
  seq_ur3_ = {
    UR3_DEFAULT_POSE,
    // { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)
    { -0.140,   -0.410,   0.180,   0.0,   -3.14,  0.0 },      // (1)
    { -0.0800,  -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (2)
    { -0.13056, -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (3)
    { -0.175,   -0.32281, 0.46642, 2.151,  0.109, -2.257 },   // (4)
    { -0.13056, -0.32030, 0.46642, 2.151,  0.109, -2.257 },   // (5)
    { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (6)
    UR3_DEFAULT_POSE,
  };


  ur3_grip_before_.clear();
  ur3_grip_after_.clear();
  rb3_grip_before_.clear();
  rb3_grip_after_.clear();
  rb3_grip_before_multi_.clear();
  rb3_grip_after_multi_.clear();

  ur3_grip_before_[0] = GripperAction::OPEN;
  ur3_grip_after_[1]  = GripperAction::CLOSE;
  ur3_grip_after_[4]  = GripperAction::OPEN;

  // rb3_grip_before_[0]  = GripperAction::HOME;
  rb3_grip_after_[0]   = GripperAction::OPEN;
  rb3_grip_before_[2]  = GripperAction::RESTART;
  rb3_grip_after_multi_[2] = { GripperAction::PINCH, GripperAction::CLOSE };
  rb3_grip_before_[3]  = GripperAction::CLOSE;
  rb3_grip_after_[8]   = GripperAction::PINCH;
  rb3_grip_after_[9]   = GripperAction::HOME;

  // 동기점
  sync_points_.clear();
  sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/2});
  sync_points_.push_back(PairSync{/*rb3_idx=*/5, /*ur3_idx=*/3, /*rb3_next=*/6, /*ur3_next=*/4});
    
  snapshot_base_sequences_hooks_sync();

  srv_objpickpoint_ = this->create_service<ObjPickSrv>(
    "objpickpoint",
    [this](const std::shared_ptr<ObjPickSrv::Request> req,
          std::shared_ptr<ObjPickSrv::Response> resp)
    {
      // (선택) 메인 시퀀스 시작 전에는 막고 싶으면 아래 3줄을 켜면 됨
      // if (services_blocked()) { resp->success=false; resp->message="main sequence not started"; return; }

      {
        std::lock_guard<std::mutex> lk(objpick_mtx_);        

        // right (RB3)
        objpick_rb3_.x  = req->right_x+0.6;     // 0.6은 ur3e 기준으로 떨어진 간격
        objpick_rb3_.y  = req->right_y;
        objpick_rb3_.z  = -0.01928;
        objpick_rb3_.rx = 1.76208441;
        objpick_rb3_.ry = 0.24801129;
        objpick_rb3_.rz = 1.77238186;

        // // const double rb_yaw = deg2rad(req->right_rz);
        // // if(rb_yaw > 1.5708)
        // // {
        // //   objpick_rb3_.rz = 90 - (rb_yaw / 2);
        // // }
        // // else objpick_rb3_.rz = rb_yaw;

        // const double rb_yaw = req->right_rz;
        // if(rb_yaw > 90)
        // {
        //   objpick_rb3_.rz = deg2rad(90 - (rb_yaw / 2));
        // }
        // else objpick_rb3_.rz = deg2rad(rb_yaw);

        objpick_ur3_.x  = req->left_x;
        objpick_ur3_.y  = req->left_y;
        objpick_ur3_.z  = 0.180;
        objpick_ur3_.rx = 0.0;
        objpick_ur3_.ry = -3.14;
        objpick_ur3_.rz = 0.0;

        // // --- 1) left_rz(도) 보정 규칙 적용 ---
        // double a_deg = req->left_rz;
        // {
        //   const double rem = std::fmod(a_deg, 90.0);
        //   const double eps = 1e-9;
        //   // 나머지가 "0이 아니면" 90 - a_deg
        //   if (std::fabs(rem) > eps) {
        //     a_deg = 90.0 - (a_deg/2);
        //   }
        // }
        // const double yaw = deg2rad(a_deg);

        // // --- 2) 기본 자세를 "회전벡터"로 정의: (0, -pi, 0) ---
        // tf2::Vector3 base_rv(0.0, -M_PI, 0.0);
        // double base_ang = base_rv.length();
        // tf2::Vector3 base_axis = (base_ang < 1e-9) ? tf2::Vector3(1,0,0) : (base_rv / base_ang);
        // tf2::Quaternion q_base(base_axis, base_ang);

        // // --- 3) yaw 회전(로컬 Z축) 델타를 합성: q = q_base * q_delta ---
        // tf2::Quaternion q_delta(tf2::Vector3(0,0,1), yaw);
        // tf2::Quaternion q = q_base * q_delta;
        // q.normalize();

        // // --- 4) quaternion -> 회전벡터(axis * angle)로 다시 변환 ---
        // tf2::Vector3 axis = q.getAxis();
        // double ang = q.getAngle();
        // tf2::Vector3 rv = axis * ang;

        // objpick_ur3_.rx = rv.x();
        // objpick_ur3_.ry = rv.y();
        // objpick_ur3_.rz = rv.z();

        objpick_ready_.store(true, std::memory_order_release);
      }

      resp->success = true;
      resp->message = "objpickpoint stored (right+left 12dof)";

      LOGP("[objpickpoint] stored. R(x=%.4f y=%.4f rz=%.4f, rx=%.4f ry=%.4f rz=%.4f) L(x=%.4f y=%.4f z=%.4f, rx=%.4f ry=%.4f rz=%.4f)",
          objpick_rb3_.x, objpick_rb3_.y, objpick_rb3_.z, objpick_rb3_.rx, objpick_rb3_.ry, objpick_rb3_.rz,
          objpick_ur3_.x,  objpick_ur3_.y,  objpick_ur3_.z, objpick_ur3_.rx, objpick_ur3_.ry, objpick_ur3_.rz);

          if (!system_ready_.load()) {
            LOGP("START skipped (called from /objpickpoint): system not ready (wait for topics/services).");
            resp->message += " | start skipped: system not ready";
            return;
          }
      
          if (mainseq_running_.load()) {
            LOGP("START skipped (called from /objpickpoint): main sequence already running.");
            resp->message += " | start skipped: already running";
            return;
          }
      
          restart_pending_.store(false);
          block_progress_ = false;
          mainseq_done_notified_ = false;
      
          this->reset_sequence_state();
          sync_reconcile_with_current_indices();
      
          mainseq_running_.store(true);
          this->start_sequence();
      
          // ★ 여기서 게이트 해제 → 이후에만 다른 서비스가 동작
          services_unlocked_.store(true, std::memory_order_release);
      
          LOGP("Main sequence started by /objpickpoint.");
          resp->message += " | main sequence started";
    }
  );

  LOGP("Service ready: /objpickpoint");

  srv_pickblock_ = this->create_service<std_srvs::srv::Empty>(
    "block_check",
    [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
          std::shared_ptr<std_srvs::srv::Empty::Response> /*resp*/)
    {
      if (services_blocked()) {  // ★ 추가
        LOGP("[GATE] PICKBLOCK ignored: main sequence not started.");
        return;
      }
      if (idx_rb3_ >= 6) {
        block_progress_ = true;
        const size_t prev_idx_rb3 = idx_rb3_;
        apply_wait_timer_ = this->create_wall_timer(
          100ms,
          [this, prev_idx_rb3]() {
            if (!rb3_waiting_ && !ur3_waiting_) {
              if (apply_wait_timer_) apply_wait_timer_->cancel();

              for (auto &sp : sync_points_) { sp.reached_rb3=false; sp.reached_ur3=false; sp.fired=false; }
              sync_inflight_.reset();
              rb3_inflight_done_ = ur3_inflight_done_ = false;
              hold_until_degcheck_ = false;
              hold_until_rotblock_ = false;
              rb3_done_all_ = false;
              rb3_seen_active_ = false;

              if (prev_idx_rb3 == 6) {
                // optional special handling
              }

              idx_rb3_ = 2;
              idx_ur3_ = 1;

              sync_reconcile_with_current_indices();

              const auto delay = std::chrono::seconds(0);
              restart_delay_timer_ = this->create_wall_timer(
                delay,
                [this]() {
                  if (restart_delay_timer_) restart_delay_timer_->cancel();
                  block_progress_ = false;

                  LOGP("RESTART delay done → FIRE pair (RB3=%zu, UR3=%zu) with BEFORE hooks", idx_rb3_, idx_ur3_);

                  sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
                  rb3_inflight_done_ = ur3_inflight_done_ = false;

                  this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/true);
                }
              );
            }
          }
        );
        LOGP("PICKBLOCK → main sequence resume scheduled (RB3=2, UR3=1)");
        return;
      }

      block_progress_ = true;
      apply_wait_timer_ = this->create_wall_timer(
        100ms,
        [this]() {
          if (!rb3_waiting_ && !ur3_waiting_) {
            if (apply_wait_timer_) apply_wait_timer_->cancel();
            for (auto &sp : sync_points_) { sp.reached_rb3=false; sp.reached_ur3=false; sp.fired=false; }
            sync_inflight_.reset();
            rb3_inflight_done_ = ur3_inflight_done_ = false;
            hold_until_degcheck_ = false;
            hold_until_rotblock_ = false;

            rb3_done_all_ = false;
            rb3_seen_active_ = false;

            // ★ RB3 훅 상태도 리셋 (before/쿨다운)
            rb3_before_already_done_for_next_ = false;
            rb3_last_hook_sent_.reset();
            if (rb3_hook_cooldown_timer_) rb3_hook_cooldown_timer_->cancel();

            size_t start_from = (seq_rb3_.size() > 2 ? 2 : 0);
            idx_rb3_ = start_from;

            sync_reconcile_with_current_indices();

            const auto delay = std::chrono::seconds(0);
            restart_delay_timer_ = this->create_wall_timer(
              delay,
              [this, start_from]() {
                if (restart_delay_timer_) restart_delay_timer_->cancel();
                block_progress_ = false;
                LOGP("RESTART delay done → resume");
                this->start_rb3_step(start_from);
                if (!ur3_waiting_) this->start_ur3_step(idx_ur3_);
              }
            );
            return;
          }
        }
      );

      LOGP("PICKBLOCK → partial restart scheduled");
    }
  );
  LOGP("Service ready: /block_check");

  srv_rotblock_ = this->create_service<RotblockSrv>(
    "angle_cmd",
    [this](const std::shared_ptr<RotblockSrv::Request> req,
            std::shared_ptr<RotblockSrv::Response> resp)
    {
      if (services_blocked()) {
        resp->success = false;
        resp->message = "main sequence not started";
        LOGP("[GATE] ROTBLOCK ignored: main sequence not started.");
        return;
      }

      // ★★★ 새로 추가: 이번 rotblock에 대한 완료 플래그 리셋 ★★★
      {
        std::lock_guard<std::mutex> lk(rotblock_mutex_);
        rotblock_done_flag_ = false;
        rotblock_result_msg_.clear();
      }

      // 기존 hold 상태 플래그
      const bool at_3_hold = hold_until_rotblock_  && (idx_rb3_ == 3);
      const bool at_4_hold = hold_until_degcheck_ && (idx_rb3_ == 4);

      // 특수 동작을 허용할 상태들
      const bool at_42 = (idx_rb3_ == 4 && idx_ur3_ == 2);
      const bool at_53 = (idx_rb3_ == 5 && idx_ur3_ == 3);

      constexpr double EPS_DEG = 0.3;

      const bool zero_rot =
          std::abs(req->r_rx_deg) <= EPS_DEG &&
          std::abs(req->r_ry_deg) <= EPS_DEG &&
          std::abs(req->r_rz_deg) <= EPS_DEG;

      // ===== ① 특수 동작: (RB3=4, UR3=2) & rotblock(0,0,0) → DEGCHECK release → (5,3) =====
      if (zero_rot && at_42 && hold_until_degcheck_) {
        // 예전 zmk_DegcheckFlag 람다와 동일한 동작
        hold_until_degcheck_ = false;
        LOGP("ROTBLOCK(0,0,0) at (RB3=4, UR3=2) → DEGCHECK release → fire (RB3=5, UR3=3)");

        idx_rb3_ = 5;
        idx_ur3_ = 3;
        sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
        rb3_inflight_done_ = ur3_inflight_done_ = false;

        // 예전처럼 BEFORE 훅 없이 바로 쏘기
        this->fire_both_immediate(idx_rb3_, idx_ur3_);

        resp->success = true;
        resp->message = "DEGCHECK released via rotblock(0,0,0) at (4,2)";
        return;
      }

      // ===== ② 특수 동작: (RB3=5, UR3=3) & rotblock(0,0,0) → sync next (6,4) =====
      if (zero_rot && at_53) {
        size_t next_rb3 = idx_rb3_;
        size_t next_ur3 = idx_ur3_;
        bool   found    = false;

        // sync_points_ 에서 (5,3)의 next (6,4)를 찾음
        for (auto &sp : sync_points_) {
          if (sp.rb3_idx == idx_rb3_ && sp.ur3_idx == idx_ur3_) {
            sp.fired  = true;          // 이후 자동 sync 재발동 방지
            next_rb3  = sp.rb3_next;
            next_ur3  = sp.ur3_next;
            found     = true;
            break;
          }
        }

        if (!found) {
          // 혹시 sync 테이블이 바뀌어도 최소한 동작은 하도록 fallback
          next_rb3 = idx_rb3_ + 1;
          next_ur3 = idx_ur3_ + 1;
        }

        hold_until_rotblock_ = false;
        hold_until_degcheck_ = false;

        idx_rb3_ = next_rb3;
        idx_ur3_ = next_ur3;
        sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
        rb3_inflight_done_ = ur3_inflight_done_ = false;

        LOGP("ROTBLOCK(0,0,0) at (RB3=5, UR3=3) → FIRE next (RB3=%zu, UR3=%zu)",
            idx_rb3_, idx_ur3_);

        // 다음 pair 실행 (BEFORE 훅 적용)
        this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/true);

        resp->success = true;
        resp->message = "Advance from (5,3) to next pair via rotblock(0,0,0)";
        return;
      }

      // ===== ③ 나머지는 원래 로직: (3 hold / 4 hold) 에서만 회전 보정 허용 =====
      if (!at_3_hold && !at_4_hold) {
        resp->success = false;
        resp->message = "Not in rotblock/degcheck hold.";
        return;
      }

      const double d_rx = deg2rad(req->r_rx_deg);
      const double d_ry = deg2rad(req->r_ry_deg);
      const double d_rz = deg2rad(req->r_rz_deg);
      tf2::Quaternion dQx, dQy, dQz;
      dQx.setRPY(d_rx, 0, 0);
      dQy.setRPY(0, d_ry, 0);
      dQz.setRPY(0, 0, d_rz);

      Pose6 p{};
      double base_rx=0, base_ry=0, base_rz=0;

      // rotblock 완료 로그용 설명 문자열 구성
      std::ostringstream desc;
      desc << std::fixed << std::setprecision(1)
          << "ΔRPY(deg)=(" << req->r_rx_deg << ", " << req->r_ry_deg << ", " << req->r_rz_deg << ")";

      if (at_3_hold) {
        // ---- (RB3=3 hold) → (RB3=4 보정 포즈) ----
        // 여기서 0,0,0을 주면 "회전 없음" → seq_rb3_[4] 그대로 전송 (요구사항 1 충족)
        p = seq_rb3_.at(4);
        base_rx = p.rx; base_ry = p.ry; base_rz = p.rz;
        tf2::Quaternion q_base; q_base.setRPY(base_rx, base_ry, base_rz); q_base.normalize();
        tf2::Quaternion q_new = q_base * dQx * dQy * dQz; q_new.normalize();
        tf2::Matrix3x3(q_new).getRPY(p.rx, p.ry, p.rz);

        // 보정 포즈 전송
        send_rb3_pose(p);

        rotblock_done_notified_ = false;

        // ★ rotblock 완료 감지 활성화 (시퀀스 상태와 독립)
        rotblock_waiting_ = true;
        rotblock_seen_active_ = false;
        t_send_rotblock_ = std::chrono::steady_clock::now();
        last_rotblock_desc_ = desc.str();

        // 3 hold 종료 → 4에서 DEGCHECK hold 진입
        hold_until_rotblock_ = false;
        idx_rb3_ = 4;
        hold_until_degcheck_ = true;
        sync_inflight_.reset();

        LOGP("ROTBLOCK: (3→4) applied, now holding for DEGCHECK at (RB3=4, UR3=2)");
      } else { // at_4_hold 이면서 zero_rot 특수조건이 아닌 경우 → 4에서 재보정만 수행
        bool used_tf = false;
        try {
          const auto tf = tf_buffer_->lookupTransform(
            rb3_base_frame_, rb3_tcp_frame_, tf2::TimePointZero);

          p.x = tf.transform.translation.x;
          p.y = tf.transform.translation.y;
          p.z = tf.transform.translation.z;

          const auto &q = tf.transform.rotation;
          tf2::Quaternion q_cur(q.x, q.y, q.z, q.w);
          q_cur.normalize();
          tf2::Matrix3x3(q_cur).getRPY(base_rx, base_ry, base_rz);

          tf2::Quaternion q_new = q_cur * dQx * dQy * dQz;
          q_new.normalize();
          tf2::Matrix3x3(q_new).getRPY(p.rx, p.ry, p.rz);

          used_tf = true;
        } catch (const tf2::TransformException &) {
          p = seq_rb3_.at(4);
          base_rx = p.rx; base_ry = p.ry; base_rz = p.rz;
          tf2::Quaternion q_base; q_base.setRPY(base_rx, base_ry, base_rz); q_base.normalize();
          tf2::Quaternion q_new = q_base * dQx * dQy * dQz; q_new.normalize();
          tf2::Matrix3x3(q_new).getRPY(p.rx, p.ry, p.rz);
        }

        send_rb3_pose(p);

        rotblock_done_notified_ = false;

        // ★ rotblock 완료 감지 활성화
        rotblock_waiting_ = true;
        rotblock_seen_active_ = false;
        t_send_rotblock_ = std::chrono::steady_clock::now();
        last_rotblock_desc_ = desc.str();

        LOGP("ROTBLOCK: reapplied @RB3=4 (%s), still in DEGCHECK hold", used_tf ? "TF" : "seq4");
      
        // ★★★ (RB3=4, UR3=2)에서 비-제로 rotblock 들어오면 즉시 5step 완료 알림 ★★★
        //    - zero_rot == false 이고
        //    - at_42 == (idx_rb3_ == 4 && idx_ur3_ == 2)
        if (!zero_rot && at_42) {
          std::ostringstream oss5;
          oss5 << "5step reached (non-zero rotblock at RB3=4, UR3=2): "
               << desc.str();
          // notify_5step_done(oss5.str());
        }
      }

      // ★★★ 여기서 rotblock 완료까지 기다리기 ★★★
      {
        std::unique_lock<std::mutex> lk(rotblock_mutex_);
        constexpr auto TIMEOUT = std::chrono::seconds(3);

        bool ok = rotblock_cv_.wait_for(
            lk, TIMEOUT,
            [this]() { return rotblock_done_flag_; });

        if (!ok) {
          resp->success = false;
          resp->message = "rotblock timeout waiting for completion";
          LOGP("ROTBLOCK: timeout waiting for completion.");
          return;
        }
      }

      const double cur_rx_deg = rad2deg(base_rx);
      const double cur_ry_deg = rad2deg(base_ry);
      const double cur_rz_deg = rad2deg(base_rz);
      const double tgt_rx_deg = rad2deg(p.rx);
      const double tgt_ry_deg = rad2deg(p.ry);
      const double tgt_rz_deg = rad2deg(p.rz);
      resp->success = true;
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1)
          << "RB3 RPY: " << cur_rx_deg << "  →  " << tgt_rx_deg
          << ", "      << cur_ry_deg << "  →  " << tgt_ry_deg
          << ", "      << cur_rz_deg << "  →  " << tgt_rz_deg;
      resp->message = oss.str();
    }
  );
  LOGP("Service ready: /angle_cmd");

  srv_start_mainseq_ = this->create_service<std_srvs::srv::Empty>(
    "start_mainseq",
    [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
          std::shared_ptr<std_srvs::srv::Empty::Response> /*resp*/)
    {
      if (!system_ready_.load()) {
        LOGP("START ignored: system not ready (wait for topics/services).");
        return;
      }
      if (mainseq_running_.load()) {
        LOGP("START ignored: main sequence already running.");
        return;
      }

      restart_pending_.store(false);
      block_progress_ = false;
      mainseq_done_notified_ = false;

      this->reset_sequence_state();
      sync_reconcile_with_current_indices();

      mainseq_running_.store(true);
      this->start_sequence();

      // ★ 여기서 게이트 해제 → 이후에만 다른 서비스가 동작
      services_unlocked_.store(true, std::memory_order_release);

      LOGP("Main sequence started by /start_mainseq.");
    }
  );

  LOGP("Service ready: /start_mainseq");

  start_timer_ = this->create_wall_timer(300ms, [this] {
    if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s) ||
        !cli_grip_open_->wait_for_service(0s) || !cli_grip_close_->wait_for_service(0s) ||
        !cli_hand_setangle_->wait_for_service(0s) ||
        (use_bimanual_srv_ && !client_bimanual_tcppos_->wait_for_service(0s))) {
      return; // 서비스 준비될 때까지 조용히 대기
    }
    const auto right_status = this->resolve_status_topic(right_fjt_base_);
    const auto left_status  = this->resolve_status_topic(left_fjt_base_);
    if (right_status.empty() || left_status.empty()) {
      LOGE("Action status topics not resolved (right=%s, left=%s)",
           right_fjt_base_.c_str(), left_fjt_base_.c_str());
      return;
    }

    sub_right_status_ = this->create_subscription<GoalStatusArray>(
      right_status, rclcpp::QoS(50),
      std::bind(&MoveSequenceClient::on_right_status, this, std::placeholders::_1));
    sub_left_status_ = this->create_subscription<GoalStatusArray>(
      left_status, rclcpp::QoS(50),
      std::bind(&MoveSequenceClient::on_left_status, this, std::placeholders::_1));

    // LOGP("All services & topics ready. Start sequence.");
    // start_timer_->cancel();

    // sync_reconcile_with_current_indices();
    // this->start_sequence();
    LOGP("All services & topics ready.");
    start_timer_->cancel();

    system_ready_.store(true);

    move_to_default_pose_on_start();

    if (auto_start_) {
      LOGP("auto_start=true → starting main sequence immediately.");
      sync_reconcile_with_current_indices();
      mainseq_running_.store(true);
      this->start_sequence();

      // ★ 자동 시작 시에도 서비스 해제
      services_unlocked_.store(true, std::memory_order_release);
    } else {
      LOGP("Waiting for /start_mainseq (Trigger) to begin the main sequence.");
    }
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

std::string MoveSequenceClient::controller_ns_from_fjt_base(const std::string &fjt_base)
{
  const std::string suf = "/follow_joint_trajectory";
  if (ends_with(fjt_base, suf)) return fjt_base.substr(0, fjt_base.size() - suf.size());
  auto pos = fjt_base.rfind('/');
  if (pos == std::string::npos) return fjt_base;
  return fjt_base.substr(0, pos);
}

void MoveSequenceClient::configure_soft_stop(double seconds)
{
  auto set_for = [this, seconds](const std::string& fjt_base) {
    rclcpp::SyncParametersClient client(this, controller_ns_from_fjt_base(fjt_base));
    if (!client.wait_for_service(std::chrono::seconds(1))) return;
    try {
      (void)client.set_parameters({ rclcpp::Parameter("stop_trajectory_duration", seconds) });
    } catch (...) {}
  };
  set_for(right_fjt_base_);
  set_for(left_fjt_base_);
}

void MoveSequenceClient::move_to_default_pose_on_start()
{
  // 서비스 준비는 start_timer_에서 이미 확인했지만, 안전하게 한 번 더 체크
  if (use_bimanual_srv_) {
    if (!client_bimanual_tcppos_->wait_for_service(0s)) {
      LOGE("[STARTUP] bimanual service not available");
      return;
    }

    auto req = std::make_shared<dual_arm_msg::srv::Movetcppos::Request>();
    // left = UR3, right = RB3
    req->left_x  = UR3_DEFAULT_POSE.x;  req->left_y  = UR3_DEFAULT_POSE.y;  req->left_z  = UR3_DEFAULT_POSE.z;
    req->left_rx = UR3_DEFAULT_POSE.rx; req->left_ry = UR3_DEFAULT_POSE.ry; req->left_rz = UR3_DEFAULT_POSE.rz;

    req->right_x  = RB3_DEFAULT_POSE.x;  req->right_y  = RB3_DEFAULT_POSE.y;  req->right_z  = RB3_DEFAULT_POSE.z;
    req->right_rx = RB3_DEFAULT_POSE.rx; req->right_ry = RB3_DEFAULT_POSE.ry; req->right_rz = RB3_DEFAULT_POSE.rz;

    LOGP("[STARTUP] Send DEFAULT via BIMANUAL async");
    client_bimanual_tcppos_->async_send_request(
      req,
      [this](rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedFuture f) {
        bool ok = false;
        std::string msg;
        try {
          auto resp = f.get();
          ok = resp && resp->success;
          msg = resp ? resp->message : "";
        } catch (...) {
          ok = false;
        }
        LOGP("[STARTUP] DEFAULT bimanual response: %s %s", ok ? "OK" : "FAIL", msg.c_str());
      }
    );
    return;
  }

  // bimanual 안 쓰는 경우도 “비동기”로만 날림 (spin_until_future_complete 금지!)
  if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s)) {
    LOGE("[STARTUP] single-arm services not available");
    return;
  }

  {
    auto req = std::make_shared<RB3Srv::Request>();
    req->right_x  = RB3_DEFAULT_POSE.x;  req->right_y  = RB3_DEFAULT_POSE.y;  req->right_z  = RB3_DEFAULT_POSE.z;
    req->right_rx = RB3_DEFAULT_POSE.rx; req->right_ry = RB3_DEFAULT_POSE.ry; req->right_rz = RB3_DEFAULT_POSE.rz;

    LOGP("[STARTUP] Send RB3 DEFAULT async");
    cli_rb3_->async_send_request(req);
  }

  {
    auto req = std::make_shared<UR3Srv::Request>();
    req->left_x  = UR3_DEFAULT_POSE.x;  req->left_y  = UR3_DEFAULT_POSE.y;  req->left_z  = UR3_DEFAULT_POSE.z;
    req->left_rx = UR3_DEFAULT_POSE.rx; req->left_ry = UR3_DEFAULT_POSE.ry; req->left_rz = UR3_DEFAULT_POSE.rz;

    LOGP("[STARTUP] Send UR3 DEFAULT async");
    cli_ur3_->async_send_request(req);
  }
}



void MoveSequenceClient::notify_rotblock_done(const std::string &msg)
{
  if (!rotblock_done_cli_) return;

  if (!rotblock_done_cli_->wait_for_service(0s)) {
    LOGE("Notify service not available: %s", rotblock_done_srv_name_.c_str());
    return;
  }

  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  rotblock_done_cli_->async_send_request(
    req,
    [this, msg](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture f) {
      bool ok = true;
      try { (void)f.get(); } catch (...) { ok = false; }
      LOGP("Notify ROTBLOCK done → %s / msg='%s'", ok ? "SENT" : "FAILED", msg.c_str());
    }
  );
}

void MoveSequenceClient::notify_mainseq_done(const std::string &msg)
{
  if (!mainseq_done_cli_) return;
  if (!mainseq_done_cli_->wait_for_service(0s)) {
    LOGE("Notify service not available: %s", mainseq_done_srv_name_.c_str());
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  mainseq_done_cli_->async_send_request(
    req,
    [this, msg](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture f) {
      bool ok = true;
      try { (void)f.get(); } catch (...) { ok = false; }
      LOGP("Notify MAINSEQ done → %s / msg='%s'", ok ? "SENT" : "FAILED", msg.c_str());
    }
  );
}

void MoveSequenceClient::maybe_notify_step4()
{
  if (step4_done_notified_) return;

  // step4의 의미: (RB3=4, UR3=2) "도달" 시점
  if (idx_rb3_ == 4 && idx_ur3_ == 2) {
    step4_done_notified_ = true;
    std::ostringstream oss4;
    oss4 << "4step reached: RB3=4, UR3=2";
    notify_4step_done(oss4.str());
  }
}

void MoveSequenceClient::notify_4step_done(const std::string &msg)
{
  if (!step4_done_cli_) return;
  if (!step4_done_cli_->wait_for_service(0s)) {
    LOGE("Notify service not available: %s", step4_done_srv_name_.c_str());
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  step4_done_cli_->async_send_request(
    req,
    [this, msg](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture f) {
      bool ok = true;
      try { (void)f.get(); } catch (...) { ok = false; }
      LOGP("Notify 4STEP done → %s / msg='%s'", ok ? "SENT" : "FAILED", msg.c_str());
    }
  );
}

void MoveSequenceClient::notify_5step_done(const std::string &msg)
{
  // 1) rotblock 완료 플래그 세팅 + condvar 깨우기
  {
    std::lock_guard<std::mutex> lk(rotblock_mutex_);
    rotblock_done_flag_ = true;
    rotblock_result_msg_ = msg;
  }
  rotblock_cv_.notify_all();

  // 2) 기존 기능(Empty 서비스 notify)은 그대로 유지
  if (!step5_done_cli_) return;
  if (!step5_done_cli_->wait_for_service(0s)) {
    LOGE("Notify service not available: %s", step5_done_srv_name_.c_str());
    return;
  }

  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  step5_done_cli_->async_send_request(
    req,
    [this, msg](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture f) {
      bool ok = true;
      try { (void)f.get(); } catch (...) { ok = false; }
      LOGP("Notify 5STEP done → %s / msg='%s'", ok ? "SENT" : "FAILED", msg.c_str());
    }
  );
}



// ---- base snapshot/restore ----
void MoveSequenceClient::snapshot_base_sequences_hooks_sync()
{
  base_seq_rb3_ = seq_rb3_;
  base_seq_ur3_ = seq_ur3_;
  base_ur3_grip_before_ = ur3_grip_before_;
  base_ur3_grip_after_  = ur3_grip_after_;
  base_rb3_grip_before_ = rb3_grip_before_;
  base_rb3_grip_after_  = rb3_grip_after_;
  base_sync_points_     = sync_points_;
  base_rb3_grip_after_multi_  = rb3_grip_after_multi_;
  base_rb3_grip_before_multi_ = rb3_grip_before_multi_;
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
  rb3_grip_after_multi_  = base_rb3_grip_after_multi_;
  rb3_grip_before_multi_ = base_rb3_grip_before_multi_;
}

// 동기점 선반영
void MoveSequenceClient::sync_reconcile_with_current_indices()
{
  for (auto &sp : sync_points_) {
    if (sp.fired) continue;
    if (idx_rb3_ >= sp.rb3_idx) sp.reached_rb3 = true;
    if (idx_ur3_ >= sp.ur3_idx) sp.reached_ur3 = true;

    if (sp.reached_rb3 && sp.reached_ur3) {
      sp.fired = true;
      if (restart_pending_.load() || block_progress_) continue;
      idx_rb3_ = sp.rb3_next;
      idx_ur3_ = sp.ur3_next;

      sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
      rb3_inflight_done_ = ur3_inflight_done_ = false;

      LOGP("SYNC-RECON → fire pair (RB3=%zu, UR3=%zu) with BEFORE hooks", idx_rb3_, idx_ur3_);
      this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/true);

      maybe_notify_step4();
      break;
    }
  }
}

// ★ HOME 시퀀스 로드 (one-shot)
void MoveSequenceClient::load_sequence_HOME()
{
  Pose6 rb3_before_home = { 0.18652, -0.27738,  0.48838,  0.15708,     0.2003638,   1.4264576};
  Pose6 ur3_before_home = {-0.13056, -0.32030, 0.46642, 2.151,       0.109,       -2.257};
  Pose6 rb3_home = RB3_DEFAULT_POSE;
  Pose6 ur3_home = UR3_DEFAULT_POSE;

  seq_rb3_.assign({ rb3_before_home, rb3_home });
  seq_ur3_.assign({ ur3_before_home, ur3_home });

  ur3_grip_before_.clear();
  ur3_grip_after_.clear();
  rb3_grip_before_.clear();
  rb3_grip_after_.clear();
  rb3_grip_before_multi_.clear();
  rb3_grip_after_multi_.clear();
  sync_points_.clear();

  ur3_grip_after_[0] = GripperAction::OPEN;
  rb3_grip_before_[0] = GripperAction::PINCH;
  rb3_grip_before_[1] = GripperAction::CLOSE;

  LOGP("HOME sequence loaded");
}

// ---- restart/cancel ----
void MoveSequenceClient::start_sequence() {
  LOGP("=== SEQ START ===");
  block_progress_ = false;
  this->prepare_and_fire_first_pair();
}
void MoveSequenceClient::reset_sequence_state() {
  rb3_waiting_ = ur3_waiting_ = false;
  rb3_seen_active_ = ur3_seen_active_ = false;
  rb3_done_all_ = ur3_done_all_ = false;
  idx_rb3_ = idx_ur3_ = 0;
  step4_done_notified_ = false;
  hold_until_degcheck_ = false;
  hold_until_rotblock_ = false;
  for (auto &sp : sync_points_) { sp.reached_rb3=false; sp.reached_ur3=false; sp.fired=false; }
  sync_inflight_.reset();
  if (sync_fire_timer_) sync_fire_timer_->cancel();
  LOGP("SEQ state reset");
}
bool MoveSequenceClient::cancel_all_goals()
{
  if (!cancel_right_cli_->wait_for_service(500ms) ||
      !cancel_left_cli_->wait_for_service(500ms)) {
    LOGE("CancelGoal service not available");
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
    LOGE("CancelGoal timeout (right=%s left=%s)", ok_r?"ok":"timeout", ok_l?"ok":"timeout");
    return false;
  }
  (void)f_r.get(); (void)f_l.get();
  std::this_thread::sleep_for(100ms);
  return true;
}

bool MoveSequenceClient::cancel_right_only()
{
  if (!cancel_right_cli_->wait_for_service(500ms)) {
    LOGE("CancelGoal RIGHT service not available");
    return false;
  }
  auto req = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  for (auto &b : req->goal_info.goal_id.uuid) b = 0;
  req->goal_info.stamp.sec = 0;
  req->goal_info.stamp.nanosec = 0;

  auto f_r = cancel_right_cli_->async_send_request(req);
  bool ok_r = (f_r.wait_for(1s) == std::future_status::ready);
  if (!ok_r) {
    LOGE("CancelGoal RIGHT timeout");
    return false;
  }
  (void)f_r.get();
  std::this_thread::sleep_for(100ms);
  return true;
}

// ---- first pair ----
void MoveSequenceClient::prepare_and_fire_first_pair() {
  if (restart_pending_.load()) return;

  LOGP("FIRE both step0 (with BEFORE hooks)");
  sync_inflight_ = std::make_pair(0u, 0u);
  rb3_inflight_done_ = ur3_inflight_done_ = false;

  // ✅ before 훅은 fire_both_now에서 처리
  this->fire_both_now(0, 0, /*apply_before_hooks=*/true);
}

// ---- concurrent fire ----
void MoveSequenceClient::fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks)
{
  if (restart_pending_.load()) return;

  if (!apply_before_hooks) {
    sync_fire_timer_ = this->create_wall_timer(
      0ms,
      [this, rb3_idx, ur3_idx](){
        if (restart_pending_.load()) { sync_fire_timer_->cancel(); return; }
        LOGP("FIRE both (RB3=%zu, UR3=%zu)", rb3_idx, ur3_idx);

        if (use_bimanual_srv_) {
          // const auto &r = seq_rb3_[rb3_idx];
          // const auto &l = seq_ur3_[ur3_idx];
          const auto r = resolve_rb3_pose(rb3_idx);
          const auto l = resolve_ur3_pose(ur3_idx);
          this->call_zmk_move_to_bimanual_tcppos_async(
            /*l*/ l.x, l.y, l.z, l.rx, l.ry, l.rz,
            /*r*/ r.x, r.y, r.z, r.rx, r.ry, r.rz);
        } else {
          this->send_ur3_step(ur3_idx);
          this->send_rb3_step(rb3_idx);
        }
        if (sync_fire_timer_) sync_fire_timer_->cancel();
      }
    );
    return;
  }

  // ---- BEFORE 훅 적용: RB3는 멀티 지원, UR3는 단일 유지
  auto rb3_seq = rb3_before_seq(rb3_idx);
  const bool do_rb3_before = !rb3_seq.empty();
  const bool do_ur3_before = ur3_grip_before_.count(ur3_idx) && ur3_grip_before_[ur3_idx] != GripperAction::NONE;

  auto pending = std::make_shared<std::atomic<int>>(0);
  if (do_rb3_before) pending->fetch_add(1);
  if (do_ur3_before) pending->fetch_add(1);

  auto fire_after_hooks = [this, rb3_idx, ur3_idx]() {
    sync_fire_timer_ = this->create_wall_timer(
      0ms,
      [this, rb3_idx, ur3_idx]() {
        if (restart_pending_.load()) { sync_fire_timer_->cancel(); return; }
        LOGP("FIRE both (after hooks) (RB3=%zu, UR3=%zu)", rb3_idx, ur3_idx);

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
        if (sync_fire_timer_) sync_fire_timer_->cancel();
      }
    );
  };

  if (pending->load() == 0) {
    // 훅이 없으면 즉시 발사
    sync_fire_timer_ = this->create_wall_timer(
      0ms, [this, rb3_idx, ur3_idx](){
        if (restart_pending_.load()) { sync_fire_timer_->cancel(); return; }
        LOGP("FIRE both (no hooks) (RB3=%zu, UR3=%zu)", rb3_idx, ur3_idx);
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
        if (sync_fire_timer_) sync_fire_timer_->cancel();
      }
    );
    return;
  }

  auto after_one_done = [this, pending, fire_after_hooks]() {
    if (pending->fetch_sub(1) == 1) {
      fire_after_hooks();
    }
  };

  if (do_rb3_before) {
    this->run_rb3_hooks_then(rb3_seq, [this, after_one_done, rb3_seq]() {
      rb3_last_hook_sent_ = rb3_seq.back();
      after_one_done();
    });
  }
  if (do_ur3_before) this->request_ur_gripper(ur3_grip_before_[ur3_idx], after_one_done);
}


void MoveSequenceClient::fire_both_immediate(size_t rb3_idx, size_t ur3_idx)
{
  if (restart_pending_.load()) return;
  LOGP("FIRE both (IMMEDIATE) RB3=%zu, UR3=%zu", rb3_idx, ur3_idx);

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
    LOGE("[BIMANUAL] service not available: %s", bimanual_srv_name_.c_str());
    return;
  }
  auto future = client_bimanual_tcppos_->async_send_request(req);
  auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s);
  if (ret != rclcpp::FutureReturnCode::SUCCESS || !future.valid()) {
    LOGE("[BIMANUAL] service timeout/invalid");
    return;
  }
  const auto resp = future.get();
  if (!(resp && resp->success)) {
    LOGE("[BIMANUAL] move_tcppos failed: %s",
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

  client_bimanual_tcppos_->async_send_request(
    req,
    [this](rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedFuture /*f*/){
      // 응답 여부와 관계없이 진행 로그는 콜백에서 별도 출력하지 않음 (간결 유지)
    });
}

// RB3 단독 포즈 송신(보정용)
void MoveSequenceClient::send_rb3_pose(const Pose6 &p)
{
  auto req = std::make_shared<RB3Srv::Request>();
  req->right_x = p.x; req->right_y = p.y; req->right_z = p.z;
  req->right_rx = p.rx; req->right_ry = p.ry; req->right_rz = p.rz;

  cli_rb3_->async_send_request(
    req,
    [this](rclcpp::Client<RB3Srv>::SharedFuture /*future*/) {});
}

// ---- RB3 BEFORE 멀티 시퀀스 구성 ----
std::vector<GripperAction> MoveSequenceClient::rb3_before_seq(size_t i) const
{
  std::vector<GripperAction> seq;
  auto itM = rb3_grip_before_multi_.find(i);
  if (itM != rb3_grip_before_multi_.end() && !itM->second.empty())
    seq.insert(seq.end(), itM->second.begin(), itM->second.end());

  auto it1 = rb3_grip_before_.find(i);
  if (it1 != rb3_grip_before_.end() && it1->second != GripperAction::NONE)
    seq.push_back(it1->second);

  return seq;
}

// ---- RB3 ----
void MoveSequenceClient::start_rb3_step(size_t i)
{
  if (restart_pending_.load() || block_progress_) return;
  if (i >= seq_rb3_.size()) { if (!rb3_done_all_) { rb3_done_all_ = true; this->maybe_finish(); } return; }

  auto seq = rb3_before_seq(i);
  if (!rb3_before_already_done_for_next_ && !seq.empty()) {
    // 멀티 훅 직렬 실행 후 동작 송신
    this->run_rb3_hooks_then(seq, [this, i, seq](){
      rb3_last_hook_sent_ = seq.back();
      this->send_rb3_step(i);
    });
  } else {
    rb3_before_already_done_for_next_ = false;
    this->send_rb3_step(i);
  }
}
void MoveSequenceClient::send_rb3_step(size_t i)
{
  if (i >= seq_rb3_.size()) return;
  // const auto &p = seq_rb3_[i];
  const auto p = resolve_rb3_pose(i);
  auto req = std::make_shared<RB3Srv::Request>();
  req->right_x = p.x; req->right_y = p.y; req->right_z = p.z;
  req->right_rx = p.rx; req->right_ry = p.ry; req->right_rz = p.rz;

  rb3_waiting_ = true;
  rb3_seen_active_ = false;
  t_send_rb3_ = std::chrono::steady_clock::now();

  LOGP("RB3 SEND step %zu", i);
  cli_rb3_->async_send_request(
    req,
    [this, i](rclcpp::Client<RB3Srv>::SharedFuture future) {
      auto resp = future.get();
      if (!(resp && resp->success)) LOGE("RB3 step %zu send failed", i);
    });
}
void MoveSequenceClient::advance_rb3_after_done(size_t just_finished_idx)
{
  if (restart_pending_.load() || block_progress_) {
    rb3_waiting_ = false;
    maybe_restart_after_idle();
    return;
  }

  std::vector<GripperAction> hook_seq;

  auto itM = rb3_grip_after_multi_.find(just_finished_idx);
  if (itM != rb3_grip_after_multi_.end() && !itM->second.empty()) {
    hook_seq.insert(hook_seq.end(), itM->second.begin(), itM->second.end());
  } else {
    auto it = rb3_grip_after_.find(just_finished_idx);
    if (it != rb3_grip_after_.end() && it->second != GripperAction::NONE) {
      hook_seq.push_back(it->second);
    }
  }

  size_t next_idx = just_finished_idx + 1;
  if (next_idx < seq_rb3_.size()) {
    auto itB = rb3_grip_before_.find(next_idx);
    if (itB != rb3_grip_before_.end() && itB->second != GripperAction::NONE) {
      hook_seq.push_back(itB->second);
    }
  }

  this->run_rb3_hooks_then(hook_seq, [this, next_idx]() {
    if (restart_pending_.load() || block_progress_) return;
    rb3_before_already_done_for_next_ = true;
    idx_rb3_ = next_idx;
    this->start_rb3_step(idx_rb3_);
  });
}

// ---- UR3e ----
void MoveSequenceClient::start_ur3_step(size_t i)
{
  if (restart_pending_.load() || block_progress_) return;
  if (i >= seq_ur3_.size()) { if (!ur3_done_all_) { ur3_done_all_ = true; this->maybe_finish(); } return; }
  if (ur3_grip_before_.count(i) && ur3_grip_before_[i] != GripperAction::NONE) {
    this->request_ur_gripper(ur3_grip_before_[i], [this, i]{ this->send_ur3_step(i); });
  } else {
    this->send_ur3_step(i);
  }
}
void MoveSequenceClient::send_ur3_step(size_t i)
{
  if (i >= seq_ur3_.size()) return;
  // const auto &p = seq_ur3_[i];
  const auto p = resolve_ur3_pose(i);
  auto req = std::make_shared<UR3Srv::Request>();
  req->left_x = p.x; req->left_y = p.y; req->left_z = p.z;
  req->left_rx = p.rx; req->left_ry = p.ry; req->left_rz = p.rz;

  ur3_waiting_ = true;
  ur3_seen_active_ = false;
  t_send_ur3_ = std::chrono::steady_clock::now();

  LOGP("UR3 SEND step %zu", i);
  cli_ur3_->async_send_request(
    req,
    [this, i](rclcpp::Client<UR3Srv>::SharedFuture future) {
      auto resp = future.get();
      if (!(resp && resp->success)) LOGE("UR3 step %zu send failed", i);
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
    this->request_ur_gripper(ur3_grip_after_[just_finished_idx], [this]{ ++idx_ur3_; this->start_ur3_step(idx_ur3_); });
  } else {
    ++idx_ur3_;
    this->start_ur3_step(idx_ur3_);
  }
}

// ---- sync / inflight ----
void MoveSequenceClient::run_after_hook_barrier_only(bool is_left, size_t finished_idx)
{
  if (is_left) {
    auto it = ur3_grip_after_.find(finished_idx);
    if (it != ur3_grip_after_.end() && it->second != GripperAction::NONE)
      this->request_ur_gripper(it->second, nullptr);
    return;
  }

  auto itM = rb3_grip_after_multi_.find(finished_idx);
  if (itM != rb3_grip_after_multi_.end() && !itM->second.empty()) {
    this->run_rb3_hooks_then(itM->second, nullptr);
    rb3_last_hook_sent_ = itM->second.back();
    return;
  }
  auto it = rb3_grip_after_.find(finished_idx);
  if (it != rb3_grip_after_.end() && it->second != GripperAction::NONE) {
    this->request_rb_hand(it->second, nullptr);
    rb3_last_hook_sent_ = it->second;
  }
}

void MoveSequenceClient::fire_rb3_only_now(size_t rb3_idx)
{
  if (restart_pending_.load()) return;

  // const auto &r = seq_rb3_[rb3_idx];
  const auto r = resolve_rb3_pose(rb3_idx);

  rb3_waiting_     = true;
  rb3_seen_active_ = false;
  t_send_rb3_      = std::chrono::steady_clock::now();

  auto req = std::make_shared<RB3Srv::Request>();
  req->right_x  = r.x;  req->right_y  = r.y;  req->right_z  = r.z;
  req->right_rx = r.rx; req->right_ry = r.ry; req->right_rz = r.rz;

  LOGP("RB3 ONLY → step %zu", rb3_idx);

  cli_rb3_->async_send_request(
    req,
    [this, rb3_idx](rclcpp::Client<RB3Srv>::SharedFuture f) {
      bool ok = false;
      try { auto resp = f.get(); ok = resp && resp->success; } catch (...) {}
      if (!ok) LOGE("RB3 ONLY step %zu send failed", rb3_idx);
    });
}

bool MoveSequenceClient::handle_sync_reached(bool is_left, size_t just_finished_idx)
{
  bool matched_any = false;
  for (auto &sp : sync_points_) {
    if (sp.fired) continue;

    if (!is_left && just_finished_idx == sp.rb3_idx) {
      sp.reached_rb3 = true; matched_any = true;
    }
    if ( is_left && just_finished_idx == sp.ur3_idx) {
      sp.reached_ur3 = true; matched_any = true;
    }

    if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
      sp.fired = true;
      if (restart_pending_.load() || block_progress_) return true;

      const size_t next_rb3 = sp.rb3_next;
      const size_t next_ur3 = sp.ur3_next;

      if (sp.rb3_idx == 3 && sp.ur3_idx == 2) {
        hold_until_rotblock_ = true;
        LOGP("SYNC (3,2) reached → HOLD for /angle_cmd");

        // ★★★ 4step 완료 알림: (RB3=3, UR3=2) 동작 완료 시 ★★★
        notify_4step_done("4step reached: (RB3=3, UR3=2) sync done");

        return true;
      }

      idx_rb3_ = next_rb3;
      idx_ur3_ = next_ur3;
      sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
      rb3_inflight_done_ = ur3_inflight_done_ = false;

      LOGP("SYNC reached → FIRE (RB3=%zu, UR3=%zu) with BEFORE hooks", idx_rb3_, idx_ur3_);
      this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/true);
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

      if (restart_pending_.load() || block_progress_) return true;

      idx_rb3_ = sp.rb3_next;
      idx_ur3_ = sp.ur3_next;

      sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
      rb3_inflight_done_ = ur3_inflight_done_ = false;

      LOGP("SYNC post-inflight → FIRE (RB3=%zu, UR3=%zu) with BEFORE hooks", idx_rb3_, idx_ur3_);
      this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/true);
      return true;
    }
  }
  return false;
}

void MoveSequenceClient::check_inflight_and_advance_after_both_done()
{
  if (!sync_inflight_.has_value()) return;
  if (!ur3_inflight_done_ && !ur3_waiting_) {
    ur3_inflight_done_ = true;
  }
  if (!(rb3_inflight_done_ && ur3_inflight_done_)) return;

  auto [rb3_idx, ur3_idx] = *sync_inflight_;
  LOGP("SYNC inflight done (RB3=%zu, UR3=%zu)", rb3_idx, ur3_idx);

  if (rb3_idx == 4 && ur3_idx == 2) {
    hold_until_degcheck_ = true;
    sync_inflight_.reset();
    LOGP("DEGCHECK hold at (RB3=4, UR3=2) — call /zmk_DegcheckFlag");
    return;
  }

  if (try_fire_next_sync_after_inflight_pair(rb3_idx, ur3_idx)) {
    return;
  }

  sync_inflight_.reset();
  rb3_inflight_done_ = ur3_inflight_done_ = false;

  if (restart_pending_.load() || block_progress_) {
    rb3_waiting_ = false;
    ur3_waiting_ = false;
    maybe_restart_after_idle();
    return;
  }

  std::vector<GripperAction> rb3_hooks;

  auto itM2 = rb3_grip_after_multi_.find(rb3_idx);
  if (itM2 != rb3_grip_after_multi_.end() && !itM2->second.empty()) {
    rb3_hooks.insert(rb3_hooks.end(), itM2->second.begin(), itM2->second.end());
  } else {
    auto itA = rb3_grip_after_.find(rb3_idx);
    if (itA != rb3_grip_after_.end() && itA->second != GripperAction::NONE) {
      rb3_hooks.push_back(itA->second);
    }
  }

  auto next_rb3 = rb3_idx + 1;
  if (next_rb3 < seq_rb3_.size()) {
    auto itB = rb3_grip_before_.find(next_rb3);
    if (itB != rb3_grip_before_.end() && itB->second != GripperAction::NONE) {
      rb3_hooks.push_back(itB->second);
    }
  }

  this->run_rb3_hooks_then(rb3_hooks, [this, next_rb3]() {
    if (restart_pending_.load() || block_progress_) return;
    rb3_before_already_done_for_next_ = true;
    idx_rb3_ = next_rb3;
    this->start_rb3_step(idx_rb3_);
    ++idx_ur3_;
    this->start_ur3_step(idx_ur3_);
  });
}

void MoveSequenceClient::maybe_finish() {
  if (rb3_done_all_ && ur3_done_all_) {
    
    if (!mainseq_done_notified_) {
      mainseq_done_notified_ = true;
      std::ostringstream oss;
      oss << "MAIN SEQUENCE DONE (RB3 idx=" << idx_rb3_
          << ", UR3 idx=" << idx_ur3_ << ")";
      notify_mainseq_done(oss.str());
    }

    mainseq_running_.store(false, std::memory_order_release);
    services_unlocked_.store(false, std::memory_order_release);

    if (auto_restart_once_) {
      LOGP("One-shot done → RESTORE & restart from 0");
      auto_restart_once_ = false;
      restore_base_sequences_hooks_sync();
      this->reset_sequence_state();
      this->start_sequence();
      return;
    }
    LOGP("=== SEQ ALL DONE ===");
    if (exit_when_done_ && !shutdown_scheduled_) {
      shutdown_scheduled_ = true;
      shutdown_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() {
          LOGP("Shutdown (exit_when_done=true)");
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
      LOGP("UR %s (Δ=%lldms) %s",
        (action == GripperAction::CLOSE ? "CLOSE" : "OPEN"),
        (long long)dt, ok ? "OK" : "FAIL");
      if (on_done) on_done();
    });
}

void MoveSequenceClient::request_rb_hand(GripperAction action, std::function<void()> on_done)
{
  if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
  if (!cli_hand_setangle_->wait_for_service(0s)) {
    if (on_done) on_done();
    return;
  }

  auto req = std::make_shared<HandSetAngleSrv::Request>();
  if (action == GripperAction::CLOSE)
  {
    // req->angle0 = 0; req->angle1 = 0; req->angle2 = 645;
    req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 645;
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
    req->angle3 = 0; req->angle4 = 300;  req->angle5 = 1000;
  }
  else if(action == GripperAction::OPEN)
  {
    req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
    req->angle3 = 1000; req->angle4 = 400; req->angle5 = 1000;
  }
  else if (action == GripperAction::RESTART)
  {
    req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
    req->angle3 = 1000; req->angle4 = 400;  req->angle5 = 1000;
  }
  req->hand_id = 1;
  req->status  = "set_angle";

  const char* tag =
    (action == GripperAction::CLOSE) ? "CLOSE" :
    (action == GripperAction::OPEN)  ? "OPEN"  :
    (action == GripperAction::PINCH) ? "PINCH" :
    (action == GripperAction::HOME)  ? "HOME"  :
    (action == GripperAction::RESTART) ? "RESTART" : "UNKNOWN";

  auto t0 = std::chrono::steady_clock::now();
  cli_hand_setangle_->async_send_request(
    req,
    [this, on_done, t0, tag](rclcpp::Client<HandSetAngleSrv>::SharedFuture future) {
      bool ok = false;
      try { auto resp = future.get(); ok = (resp != nullptr); } catch (...) {}
      auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0).count();
      LOGP("RB HAND %s (Δ=%lldms) %s", tag, (long long)dt, ok ? "CALLED" : "FAIL");
      if (on_done) on_done();
    }
  );
}

// ---- 직렬 RB3 훅 실행 유틸 ----
void MoveSequenceClient::run_rb3_hooks_then(std::vector<GripperAction> seq, std::function<void()> then)
{
  if (seq.empty()) { if (then) then(); return; }

  auto self = std::make_shared<std::function<void(size_t)>>();
  *self = [this, self, seq, then](size_t i) {
    if (i >= seq.size()) { if (then) then(); return; }

    GripperAction act = seq[i];

    auto call_next = [this, self, seq, then, i]() {
      this->request_rb_hand(seq[i], [this, self, seq, then, i]() {
        rb3_last_hook_sent_ = seq[i];
        (*self)(i+1);
      });
    };

    if (rb3_last_hook_sent_.has_value() && rb3_last_hook_sent_.value() == act) {
      rb3_hook_cooldown_timer_ = this->create_wall_timer(
        120ms, [this, call_next]() {
          if (rb3_hook_cooldown_timer_) rb3_hook_cooldown_timer_->cancel();
          call_next();
        }
      );
    } else {
      call_next();
    }
  };

  (*self)(0);
}


// ---- status callbacks ----
void MoveSequenceClient::on_right_status(GoalStatusArray::SharedPtr msg)
{
  const bool active = any_active(*msg);

  // ★ rotblock 보정 동작 완료 감지 (시퀀스와 독립)
  if (rotblock_waiting_) {
    if (active && !rotblock_seen_active_) {
      rotblock_seen_active_ = true;
      t_active_rotblock_ = std::chrono::steady_clock::now();
    }
    if (rotblock_seen_active_ && !active) {
      auto now = std::chrono::steady_clock::now();
      auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_rotblock_).count();
      auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_rotblock_).count();
      LOGP("ROTBLOCK DONE %s (Δsend→done=%lldms, Δactive→done=%lldms)",
           last_rotblock_desc_.c_str(), (long long)d1, (long long)d2);

      if (idx_rb3_ == 4 && hold_until_degcheck_ && !rotblock_done_notified_) {
        rotblock_done_notified_ = true;
        std::ostringstream oss;
        oss << "ROTBLOCK DONE @RB3=4 (Δsend→done=" << d1 << "ms, Δactive→done=" << d2 << "ms)";
        notify_rotblock_done(oss.str());

        // ★★★ 5step 완료 알림: (RB3=4, UR3=2) rotblock 동작이 실제로 끝났을 때 ★★★
        std::ostringstream oss5;
        oss5 << "5step reached: RB3=4, UR3=" << idx_ur3_
             << " (rotblock done, " << last_rotblock_desc_ << ")";
        notify_5step_done(oss5.str());
      }

      rotblock_waiting_ = false;
      rotblock_seen_active_ = false;
      last_rotblock_desc_.clear();
      // rotblock 완료는 hold 상태/인덱스에는 영향을 주지 않음
    }
  }

  if (rb3_waiting_) {
    if (active && !rb3_seen_active_) {
      rb3_seen_active_ = true;
      t_active_rb3_ = std::chrono::steady_clock::now();
    }
    if (rb3_seen_active_ && !active) {
      rb3_waiting_ = false;
      auto now = std::chrono::steady_clock::now();
      auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_rb3_).count();
      auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_rb3_).count();
      LOGP("RB3 DONE idx=%zu (Δsend → done=%lldms, Δactive → done=%lldms)",
           idx_rb3_, (long long)d1, (long long)d2);

      if (sync_inflight_.has_value() && idx_rb3_ == sync_inflight_->first) {
        rb3_inflight_done_ = true;
        this->check_inflight_and_advance_after_both_done();
        return;
      }
      if (this->handle_sync_reached(false, idx_rb3_)) return;

      this->advance_rb3_after_done(idx_rb3_);
      maybe_notify_step4();
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
    }
    if (ur3_seen_active_ && !active) {
      ur3_waiting_ = false;
      auto now = std::chrono::steady_clock::now();
      auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_send_ur3_).count();
      auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_active_ur3_).count();
      LOGP("UR3 DONE idx=%zu (Δsend → done=%lldms, Δactive → done=%lldms)",
           idx_ur3_, (long long)d1, (long long)d2);

      if (sync_inflight_.has_value() && idx_ur3_ == sync_inflight_->second) {
        this->run_after_hook_barrier_only(true, idx_ur3_);
        ur3_inflight_done_ = true;
        this->check_inflight_and_advance_after_both_done();
        return;
      }
      if (this->handle_sync_reached(true, idx_ur3_)) return;

      this->advance_ur3_after_done(idx_ur3_);
      maybe_notify_step4();
      maybe_restart_after_idle();
    }
  }
}

// ---- soft-restart helpers ----
void MoveSequenceClient::request_soft_restart()
{
  restart_pending_.store(true);
  block_progress_ = true;

  if (!restart_watchdog_ || restart_watchdog_->is_canceled()) {
    restart_watchdog_ = this->create_wall_timer(
      100ms, [this]() {
        if (!restart_pending_.load()) { restart_watchdog_->cancel(); return; }
        if (!rb3_waiting_ && !ur3_waiting_) {
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

Pose6 MoveSequenceClient::resolve_rb3_pose(size_t i) const
{
  // RB3는 step 2에서 objpick 적용
  if (i == 2 && objpick_ready_.load(std::memory_order_acquire)) {
    std::lock_guard<std::mutex> lk(objpick_mtx_);
    LOGP("RB3 step2 uses OBJPICK");
    return objpick_rb3_;
  }
  return seq_rb3_.at(i);
}

Pose6 MoveSequenceClient::resolve_ur3_pose(size_t i) const
{
  // UR3는 step 1에서 objpick 적용
  if (i == 1 && objpick_ready_.load(std::memory_order_acquire)) {
    std::lock_guard<std::mutex> lk(objpick_mtx_);
    LOGP("UR3 step1 uses OBJPICK");
    return objpick_ur3_;
  }
  return seq_ur3_.at(i);
}

void MoveSequenceClient::do_restart_now()
{
  if (!restart_pending_.load()) return;
  LOGP("SOFT-RESTART now");
  this->reset_sequence_state();
  restart_pending_.store(false);
  block_progress_ = false;
  this->start_sequence();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveSequenceClient>());
  rclcpp::shutdown();
  return 0;
}

