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
//   // active (현재 적용 중) 세트
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

//   // === (추가) base 스냅샷: 메인 시퀀스/훅/동기점 ===
//   std::vector<Pose6> base_seq_rb3_, base_seq_ur3_;
//   std::unordered_map<size_t, GripperAction> base_ur3_grip_before_;
//   std::unordered_map<size_t, GripperAction> base_ur3_grip_after_;
//   std::unordered_map<size_t, GripperAction> base_rb3_grip_before_;
//   std::unordered_map<size_t, GripperAction> base_rb3_grip_after_;
//   std::vector<PairSync> base_sync_points_;

//   // === add this ===
//   bool auto_restart_once_ = false;   // A/B 완료 후 1회만 메인 시퀀스로 자동 복귀

//   // ---- soft-restart flags ----
//   std::atomic<bool> restart_pending_{false};
//   bool block_progress_{false};
//   rclcpp::TimerBase::SharedPtr restart_watchdog_;

//   rclcpp::TimerBase::SharedPtr apply_wait_timer_; // 재시퀀스 적용 대기 타이머

//   // ---------- helpers (decl) ----------
//   std::string ts() const;
//   static bool ends_with(const std::string &s, const std::string &suffix);
//   static bool any_active(const GoalStatusArray &arr);
//   std::string resolve_status_topic(const std::string &base);

//   void start_sequence();
//   void reset_sequence_state();
//   bool cancel_all_goals();

//   // === base 스냅샷/복원 ===
//   void snapshot_base_sequences_hooks_sync();
//   void restore_base_sequences_hooks_sync();

//   void load_sequence_A();  // 조건 충족 시 1회 실행
//   void load_sequence_B();  // 조건 미충족 시 1회 실행
//   void apply_sequence_and_soft_restart(bool use_A);

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
//   void request_soft_restart();
//   void maybe_restart_after_idle();
//   void do_restart_now();
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

//   // ---- 초기 메인 시퀀스/훅/동기점 정의 ----
//   seq_rb3_ = {
//     {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (0)
//     {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (1)
//     {-0.04642, -0.44152, 0.120, 1.76208441,    0.24801129,  1.77238186}, // (2)
//     { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (3)
//     { 0.07944, -0.22198, 0.41277, 1.84917634, -1.4379768,   1.4158111},  // (4)
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

//   // gripper hooks (메인)
//   ur3_grip_before_.clear();
//   ur3_grip_after_.clear();
//   rb3_grip_before_.clear();
//   rb3_grip_after_.clear();
//   ur3_grip_before_[0] = GripperAction::OPEN;
//   ur3_grip_after_[1]  = GripperAction::CLOSE;
//   ur3_grip_after_[4]  = GripperAction::OPEN;

//   rb3_grip_after_[0]  = GripperAction::OPEN;
//   rb3_grip_before_[2] = GripperAction::PINCH;
//   rb3_grip_after_[2]  = GripperAction::CLOSE;
//   rb3_grip_before_[3] = GripperAction::CLOSE;
//   rb3_grip_after_[8]  = GripperAction::PINCH;
//   rb3_grip_after_[9]  = GripperAction::HOME;

//   // sync points (메인)
//   sync_points_.clear();
//   sync_points_.push_back(PairSync{/*rb3_idx=*/5, /*ur3_idx=*/3, /*rb3_next=*/6, /*ur3_next=*/4});
//   sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/2});

//   // === base 스냅샷 저장 (메인으로 복귀 때 사용) ===
//   snapshot_base_sequences_hooks_sync();

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

//   // ★ pickblock 조건 서비스
//   srv_pickblock_ = this->create_service<std_srvs::srv::Trigger>(
//     "zmk_pickblockchkFlag",
//     [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
//           std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
//     {
//       const bool use_A = (idx_rb3_ >= 6 && idx_ur3_ >= 4);
//       RCLCPP_WARN(this->get_logger(),
//         "[%s] [PICKBLOCK] request at RB3=%zu, UR3=%zu → %s",
//         this->ts().c_str(), idx_rb3_, idx_ur3_,
//         use_A ? "LOAD SEQ-A (once) & RESTART" : "LOAD SEQ-B (once) & RESTART");

//       // 이후 진행 잠금
//       block_progress_ = true;

//       auto apply_now = [this, use_A]() {
//         apply_sequence_and_soft_restart(use_A);
//       };

//       // 이미 양팔 idle이면 즉시 적용
//       if (!rb3_waiting_ && !ur3_waiting_) {
//         apply_now();
//       } else {
//         // idle 될 때까지 100ms 폴링
//         apply_wait_timer_ = this->create_wall_timer(
//           100ms,
//           [this, apply_now]() {
//             if (!rb3_waiting_ && !ur3_waiting_) {
//               if (apply_wait_timer_) apply_wait_timer_->cancel();
//               apply_now();
//             }
//           }
//         );
//       }

//       resp->success = true;
//       resp->message = use_A
//         ? "Sequence-A will run once then return to MAIN from step 0"
//         : "Sequence-B will run once then return to MAIN from step 0";
//     }
//   );

//   RCLCPP_INFO(get_logger(), "Service server ready: /zmk_pickblockchkFlag");

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

// // ---- base 스냅샷/복원 ----
// void MoveSequenceClient::snapshot_base_sequences_hooks_sync()
// {
//   base_seq_rb3_ = seq_rb3_;
//   base_seq_ur3_ = seq_ur3_;
//   base_ur3_grip_before_ = ur3_grip_before_;
//   base_ur3_grip_after_  = ur3_grip_after_;
//   base_rb3_grip_before_ = rb3_grip_before_;
//   base_rb3_grip_after_  = rb3_grip_after_;
//   base_sync_points_     = sync_points_;
//   RCLCPP_INFO(this->get_logger(), "[%s] [BASE] snapshot stored.", this->ts().c_str());
// }

// void MoveSequenceClient::restore_base_sequences_hooks_sync()
// {
//   seq_rb3_ = base_seq_rb3_;
//   seq_ur3_ = base_seq_ur3_;
//   ur3_grip_before_ = base_ur3_grip_before_;
//   ur3_grip_after_  = base_ur3_grip_after_;
//   rb3_grip_before_ = base_rb3_grip_before_;
//   rb3_grip_after_  = base_rb3_grip_after_;
//   sync_points_     = base_sync_points_;
//   RCLCPP_INFO(this->get_logger(), "[%s] [BASE] restored MAIN sequences/hooks/sync.", this->ts().c_str());
// }

// // ---- restart/cancel ----
// void MoveSequenceClient::start_sequence() {
//   RCLCPP_INFO(this->get_logger(), "[%s] === Sequence PREP ===", this->ts().c_str());
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

// void MoveSequenceClient::apply_sequence_and_soft_restart(bool use_A)
// {
//   if (use_A) load_sequence_A();
//   else       load_sequence_B();

//   // A/B는 1회만 실행하고 종료되면 메인으로 복귀
//   auto_restart_once_ = true;
//   exit_when_done_ = false;

//   // 상태 초기화 후 0번부터 재시작 (A/B를 active로)
//   this->reset_sequence_state();
//   block_progress_ = false;
//   this->start_sequence();
// }

// void MoveSequenceClient::load_sequence_A()
// {
//   // === 새 시퀀스 A (간단 예시) ===
//   seq_rb3_.assign({
//     {-0.04381, -0.45377, 0.13585, 1.75492856,  0.19338248,  1.8102555},  // (0)
//     {-0.0407,  -0.44568, 0.31162, 1.7228145,   0.17715092,  1.80205245}, // (1)
//     {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (2)
//   });
//   seq_ur3_.assign({
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)
//     { -0.140,   -0.410,   0.175,   0.0,   -3.14,  0.0 },      // (1)
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (2)
//   });

//   // 훅 초기화 (A에서는 훅/동기점 사용 안 함)
//   ur3_grip_before_.clear();
//   ur3_grip_after_.clear();
//   rb3_grip_before_.clear();
//   rb3_grip_after_.clear();

//   ur3_grip_after_[1]  = GripperAction::OPEN;
//   rb3_grip_after_[0]  = GripperAction::PINCH;
//   rb3_grip_after_[1]  = GripperAction::HOME;

//   sync_points_.clear();

//   RCLCPP_INFO(this->get_logger(), "[%s] SEQ-A loaded (one-shot).", this->ts().c_str());
// }

// void MoveSequenceClient::load_sequence_B()
// {
//   // === 새 시퀀스 B (간단 예시) ===
//   seq_rb3_.assign({
//     {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (0)
//     {-0.04642, -0.44152, 0.121, 1.76208441,  0.24801129,  1.77238186}, // (1)
//     {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (2)
//     {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (3)
//   });
//   seq_ur3_.assign({
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (0)
//     { -0.140,   -0.410,   0.175,   0.0,   -3.14,  0.0 },      // (1)
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },      // (2)
//   });

//   ur3_grip_before_.clear();
//   ur3_grip_after_.clear();
//   rb3_grip_before_.clear();
//   rb3_grip_after_.clear();

//   // 예시로 간단 훅만
//   ur3_grip_after_[1]  = GripperAction::OPEN;
//   rb3_grip_after_[1]  = GripperAction::PINCH;
//   rb3_grip_after_[2]  = GripperAction::PINCH;
//   rb3_grip_after_[3]  = GripperAction::HOME;

//   sync_points_.clear();

//   RCLCPP_WARN(this->get_logger(), "[%s] SEQ-B loaded (one-shot).", this->ts().c_str());
// }

// // ---- soft-restart helpers ----
// void MoveSequenceClient::request_soft_restart()
// {
//   restart_pending_.store(true);
//   block_progress_ = true;

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

// bool MoveSequenceClient::try_fire_next_sync_after_inflight_pair(size_t just_rb3_idx, size_t just_ur3_idx)
// {
//   for (auto &sp : sync_points_) {
//     if (sp.rb3_idx == just_rb3_idx && sp.ur3_idx == just_ur3_idx) {
//       if (sp.fired) return false;
//       sp.fired = true;

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

//   if (rb3_idx == 4 && ur3_idx == 2) {
//     hold_until_degcheck_ = true;
//     sync_inflight_.reset();
//     RCLCPP_INFO(this->get_logger(), "[%s] [DEGCHECK] Holding at (RB3=4, UR3=2). Call `/zmk_DegcheckFlag`.", this->ts().c_str());
//     return;
//   }

//   if (try_fire_next_sync_after_inflight_pair(rb3_idx, ur3_idx)) {
//     return;
//   }

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
//     if (auto_restart_once_) {
//       RCLCPP_INFO(this->get_logger(),
//         "[%s] One-shot sequence finished → RESTORE MAIN & restart from step 0.", this->ts().c_str());
//       auto_restart_once_ = false;   // 1회만
//       // 메인 시퀀스로 복귀
//       restore_base_sequences_hooks_sync();
//       this->reset_sequence_state();
//       this->start_sequence();
//       return;
//     }
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
//                     action == GripperAction::OPEN  ? "OPEN"  :
//                     action == GripperAction::PINCH ? "PINCH" : "HOME");

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










//-------------------------------20251015_rb3만 시퀀스 재동작, 7번째부터 home위치로 가는것까지 했는데 Test 필요----------------------

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
// #include <rclcpp/parameter_client.hpp>
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

//   // base snapshot (main)
//   std::vector<Pose6> base_seq_rb3_, base_seq_ur3_;
//   std::unordered_map<size_t, GripperAction> base_ur3_grip_before_;
//   std::unordered_map<size_t, GripperAction> base_ur3_grip_after_;
//   std::unordered_map<size_t, GripperAction> base_rb3_grip_before_;
//   std::unordered_map<size_t, GripperAction> base_rb3_grip_after_;
//   std::vector<PairSync> base_sync_points_;

//   // soft-restart flags
//   std::atomic<bool> restart_pending_{false};
//   bool block_progress_{false};
//   rclcpp::TimerBase::SharedPtr restart_watchdog_;

//   rclcpp::TimerBase::SharedPtr apply_wait_timer_;

//   // one-shot 완료 후 메인으로 복귀 여부
//   bool auto_restart_once_ = false;

//   // ---------- helpers (decl) ----------
//   std::string ts() const;
//   static bool ends_with(const std::string &s, const std::string &suffix);
//   static bool any_active(const GoalStatusArray &arr);
//   std::string resolve_status_topic(const std::string &base);

//   // controller param setup
//   std::string controller_ns_from_fjt_base(const std::string &fjt_base);
//   void configure_soft_stop(double seconds);

//   void start_sequence();
//   void reset_sequence_state();
//   bool cancel_all_goals();
//   bool cancel_right_only();

//   void snapshot_base_sequences_hooks_sync();
//   void restore_base_sequences_hooks_sync();

//   // 동기점 보정: 현재 인덱스 기준으로 reached_* 선반영
//   void sync_reconcile_with_current_indices();

//   // 홈/원샷 시퀀스 로더
//   void load_sequence_HOME();

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
//   void request_soft_restart();
//   void maybe_restart_after_idle();
//   void do_restart_now();
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

//   // soft stop 0.5s
//   configure_soft_stop(0.5);

//   // ---- main sequences/hooks/sync ----
//   seq_rb3_ = {
//     {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (0)
//     {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (1)
//     {-0.04642, -0.44152, 0.120, 1.76208441,    0.24801129,  1.77238186}, // (2)
//     { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (3)
//     { 0.07944, -0.22198, 0.41277, 1.84917634, -1.4379768,   1.4158111},  // (4)
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

//   ur3_grip_before_.clear();
//   ur3_grip_after_.clear();
//   rb3_grip_before_.clear();
//   rb3_grip_after_.clear();
//   ur3_grip_before_[0] = GripperAction::OPEN;
//   ur3_grip_after_[1]  = GripperAction::CLOSE;
//   ur3_grip_after_[4]  = GripperAction::OPEN;

//   rb3_grip_after_[0]  = GripperAction::OPEN;
//   rb3_grip_before_[2] = GripperAction::PINCH;
//   rb3_grip_after_[2]  = GripperAction::CLOSE;
//   rb3_grip_before_[3] = GripperAction::CLOSE;
//   rb3_grip_after_[8]  = GripperAction::PINCH;
//   rb3_grip_after_[9]  = GripperAction::HOME;

//   // 동기점 유지
//   sync_points_.clear();
//   sync_points_.push_back(PairSync{/*rb3_idx=*/5, /*ur3_idx=*/3, /*rb3_next=*/6, /*ur3_next=*/4});
//   sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/2});

//   snapshot_base_sequences_hooks_sync();

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

//   // ★ pickblock: 조건 분기
//   // - RB3 idx >= 6  → 두 로봇 HOME 시퀀스 1회 실행 후 **종료**
//   // - RB3 idx <  6  → 기존 동작 (UR3 유지, RB3 0부터 재시작)
//   srv_pickblock_ = this->create_service<std_srvs::srv::Trigger>(
//     "zmk_pickblockchkFlag",
//     [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
//           std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
//     {
//       if (idx_rb3_ >= 6) {
//         RCLCPP_WARN(this->get_logger(),
//           "[%s] [PICKBLOCK] RB3 idx=%zu >= 6 → RUN HOME sequence for BOTH (one-shot) and EXIT",
//           this->ts().c_str(), idx_rb3_);

//         block_progress_ = true;

//         // 두 팔 모두 취소(소프트 스톱 파라미터는 이미 적용됨)
//         // if (!cancel_all_goals()) {
//         //   RCLCPP_ERROR(this->get_logger(), "[%s] [PICKBLOCK] Cancel both failed (continue anyway)", this->ts().c_str());
//         // }

//         // 양팔 idle 대기 후 홈 시퀀스 적용
//         apply_wait_timer_ = this->create_wall_timer(
//           100ms,
//           [this]() {
//             // 양팔이 모두 idle일 때만 홈 시퀀스 적용
//             if (!rb3_waiting_ && !ur3_waiting_) {
//               if (apply_wait_timer_) apply_wait_timer_->cancel();

//               // 홈 시퀀스 로드
//               load_sequence_HOME();

//               // 메인 복귀 금지 & 완료 후 종료
//               auto_restart_once_ = false;
//               exit_when_done_ = true;

//               // 상태 리셋 후 시작
//               this->reset_sequence_state();
//               block_progress_ = false;

//               RCLCPP_INFO(this->get_logger(), "[%s] [PICKBLOCK] Start HOME sequence after current step (will exit when done)", this->ts().c_str());
//               this->start_sequence();
//             }
//           }
//         );

//         resp->success = true;
//         resp->message = "HOME sequence (both) will run once, then the node will exit.";
//         return;
//       }

//       // --- 기본(기존) 분기: UR3 유지, RB3 0부터 ---
//       RCLCPP_WARN(this->get_logger(),
//         "[%s] [PICKBLOCK] Partial restart: keep UR3 idx=%zu, restart RB3 from 0",
//         this->ts().c_str(), idx_ur3_);

//       block_progress_ = true;

//       // if (!cancel_right_only()) {
//       //   RCLCPP_ERROR(this->get_logger(), "[%s] [PICKBLOCK] Cancel RB3 failed", this->ts().c_str());
//       // }

//       apply_wait_timer_ = this->create_wall_timer(
//         100ms,
//         [this]() {
//           // RB3가 idle이면 재시작 적용 (UR3는 계속 진행 중이어도 됨)
//           if (!rb3_waiting_) {
//             if (apply_wait_timer_) apply_wait_timer_->cancel();

//             // 상태 초기화(UR3는 유지), 동기점 재보정 준비
//             for (auto &sp : sync_points_) { sp.reached_rb3=false; sp.reached_ur3=false; sp.fired=false; }
//             sync_inflight_.reset();
//             rb3_inflight_done_ = ur3_inflight_done_ = false;
//             hold_until_degcheck_ = false;

//             rb3_done_all_ = false;
//             rb3_seen_active_ = false;

//             size_t start_from = (seq_rb3_.size() > 1 ? 1 : 0);
//             idx_rb3_ = start_from;

//             // 현재 인덱스로 동기점 선반영
//             sync_reconcile_with_current_indices();

//             block_progress_ = false;

//             RCLCPP_INFO(this->get_logger(),
//               "[%s] [PICKBLOCK] RB3 start from %zu, UR3 keep idx=%zu (UR3 waiting=%s)",
//               this->ts().c_str(), idx_rb3_, idx_ur3_, ur3_waiting_ ? "true" : "false");

//             this->start_rb3_step(start_from);

//             if (!ur3_waiting_) {
//               this->start_ur3_step(idx_ur3_);
//             }
//           }
//         }
//       );

//       resp->success = true;
//       resp->message = "Partial restart applied: RB3 from 0, UR3 keeps current step.";
//     }
//   );

//   RCLCPP_INFO(get_logger(), "Service server ready: /zmk_pickblockchkFlag");

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

//     // 시작 직전 동기점 선반영
//     sync_reconcile_with_current_indices();
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

// // controller ns from fjt base
// std::string MoveSequenceClient::controller_ns_from_fjt_base(const std::string &fjt_base)
// {
//   const std::string suf = "/follow_joint_trajectory";
//   if (ends_with(fjt_base, suf)) {
//     return fjt_base.substr(0, fjt_base.size() - suf.size());
//   }
//   auto pos = fjt_base.rfind('/');
//   if (pos == std::string::npos) return fjt_base;
//   return fjt_base.substr(0, pos);
// }

// // soft stop (ROS 2 Humble 시그니처)
// void MoveSequenceClient::configure_soft_stop(double seconds)
// {
//   auto set_for = [this, seconds](const std::string& fjt_base) {
//     auto ctrl_ns = controller_ns_from_fjt_base(fjt_base);
//     rclcpp::SyncParametersClient client(this, ctrl_ns);
//     if (!client.wait_for_service(std::chrono::seconds(1))) {
//       RCLCPP_WARN(this->get_logger(),
//         "[%s] Soft-stop: parameter service not available for %s (skip)",
//         this->ts().c_str(), ctrl_ns.c_str());
//       return;
//     }
//     try {
//       auto results = client.set_parameters({
//         rclcpp::Parameter("stop_trajectory_duration", seconds)
//       });
//       bool ok = results.empty() ? false : results.front().successful;
//       if (ok) {
//         RCLCPP_INFO(this->get_logger(),
//           "[%s] Soft-stop applied: %s.stop_trajectory_duration=%.3f s",
//           this->ts().c_str(), ctrl_ns.c_str(), seconds);
//       } else {
//         RCLCPP_WARN(this->get_logger(),
//           "[%s] Soft-stop set failed for %s: %s",
//           this->ts().c_str(), ctrl_ns.c_str(),
//           results.empty() ? "no result" : results.front().reason.c_str());
//       }
//     } catch (const std::exception& e) {
//       RCLCPP_WARN(this->get_logger(),
//         "[%s] Soft-stop set threw for %s: %s",
//         this->ts().c_str(), ctrl_ns.c_str(), e.what());
//     }
//   };
//   set_for(right_fjt_base_);
//   set_for(left_fjt_base_);
// }

// // ---- base snapshot/restore ----
// void MoveSequenceClient::snapshot_base_sequences_hooks_sync()
// {
//   base_seq_rb3_ = seq_rb3_;
//   base_seq_ur3_ = seq_ur3_;
//   base_ur3_grip_before_ = ur3_grip_before_;
//   base_ur3_grip_after_  = ur3_grip_after_;
//   base_rb3_grip_before_ = rb3_grip_before_;
//   base_rb3_grip_after_  = rb3_grip_after_;
//   base_sync_points_     = sync_points_;
//   RCLCPP_INFO(this->get_logger(), "[%s] [BASE] snapshot stored.", this->ts().c_str());
// }

// void MoveSequenceClient::restore_base_sequences_hooks_sync()
// {
//   seq_rb3_ = base_seq_rb3_;
//   seq_ur3_ = base_seq_ur3_;
//   ur3_grip_before_ = base_ur3_grip_before_;
//   ur3_grip_after_  = base_ur3_grip_after_;
//   rb3_grip_before_ = base_rb3_grip_before_;
//   rb3_grip_after_  = base_rb3_grip_after_;
//   sync_points_     = base_sync_points_;
//   RCLCPP_INFO(this->get_logger(), "[%s] [BASE] restored MAIN sequences/hooks/sync.", this->ts().c_str());
// }

// // 동기점 선반영
// void MoveSequenceClient::sync_reconcile_with_current_indices()
// {
//   for (auto &sp : sync_points_) {
//     if (sp.fired) continue;
//     if (idx_rb3_ >= sp.rb3_idx) sp.reached_rb3 = true;
//     if (idx_ur3_ >= sp.ur3_idx) sp.reached_ur3 = true;

//     if (sp.reached_rb3 && sp.reached_ur3) {
//       sp.fired = true;
//       if (restart_pending_.load() || block_progress_) {
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC-RECON] restart pending → skip fire", this->ts().c_str());
//         continue;
//       }
//       idx_rb3_ = sp.rb3_next;
//       idx_ur3_ = sp.ur3_next;

//       sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;

//       RCLCPP_INFO(this->get_logger(),
//         "[%s] [SYNC-RECON] BOTH already ≥ targets → FIRE (RB3=%zu, UR3=%zu)",
//         this->ts().c_str(), idx_rb3_, idx_ur3_);
//       this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
//       break;
//     }
//   }
// }

// // ★ HOME 시퀀스 로드 (one-shot)
// void MoveSequenceClient::load_sequence_HOME()
// {
//   // 메인 시퀀스의 "home" 포즈를 그대로 사용 (RB3 idx 10, UR3 idx 6)
//   // Pose6 rb3_before_home = {-0.0407,  -0.44568, 0.31162, 1.7228145,   0.17715092,  1.80205245};
//   Pose6 rb3_before_home = { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576};
//   Pose6 rb3_home =        {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708};
//   Pose6 ur3_before_home = {-0.13056, -0.32030, 0.46642, 2.151,       0.109,       -2.257};
//   Pose6 ur3_home =        {-0.140,   -0.410,   0.350,   0.0,         -3.14,       0.0};

//   seq_rb3_.assign({
//     rb3_before_home,
//     rb3_home
//   });
//   seq_ur3_.assign({
//     ur3_before_home,
//     ur3_home
//   });

//   // 훅/동기점 초기화
//   ur3_grip_before_.clear();
//   ur3_grip_after_.clear();
//   rb3_grip_before_.clear();
//   rb3_grip_after_.clear();
//   sync_points_.clear();

//   RCLCPP_INFO(this->get_logger(), "[%s] HOME sequence loaded (one-shot).", this->ts().c_str());
// }

// // ---- restart/cancel ----
// void MoveSequenceClient::start_sequence() {
//   RCLCPP_INFO(this->get_logger(), "[%s] === Sequence PREP ===", this->ts().c_str());
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

// bool MoveSequenceClient::cancel_right_only()
// {
//   if (!cancel_right_cli_->wait_for_service(500ms)) {
//     RCLCPP_ERROR(this->get_logger(), "[%s] CancelGoal RIGHT service not available", this->ts().c_str());
//     return false;
//   }
//   auto req = std::make_shared<action_msgs::srv::CancelGoal::Request>();
//   for (auto &b : req->goal_info.goal_id.uuid) b = 0;
//   req->goal_info.stamp.sec = 0;
//   req->goal_info.stamp.nanosec = 0;

//   auto f_r = cancel_right_cli_->async_send_request(req);
//   bool ok_r = (f_r.wait_for(1s) == std::future_status::ready);
//   if (!ok_r) {
//     RCLCPP_ERROR(this->get_logger(), "[%s] CancelGoal RIGHT timeout", this->ts().c_str());
//     return false;
//   }
//   try {
//     auto r = f_r.get();
//     RCLCPP_INFO(this->get_logger(), "[%s] CancelGoal RIGHT ret(code=%d)", this->ts().c_str(), r->return_code);
//   } catch (...) {
//     RCLCPP_WARN(this->get_logger(), "[%s] CancelGoal RIGHT exception (ignored)", this->ts().c_str());
//   }
//   std::this_thread::sleep_for(100ms);
//   return true;
// }

// // ---- soft-restart helpers ----
// void MoveSequenceClient::request_soft_restart()
// {
//   restart_pending_.store(true);
//   block_progress_ = true;

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
//   RCLCPP_WARN(this->get_logger(), "[%s] [SOFT-RESTART] Reset & start from 0", this->ts().c_str());
//   this->reset_sequence_state();
//   restart_pending_.store(false);
//   block_progress_ = false;
//   this->start_sequence();
// }

// // ---- first pair ----
// void MoveSequenceClient::prepare_and_fire_first_pair() {
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
//       if (idx_ur3_ >= sp.ur3_idx) sp.reached_ur3 = true;  // 이미 ≥면 도달
//     }
//     if ( is_left && just_finished_idx == sp.ur3_idx) {
//       sp.reached_ur3 = true; matched_any = true;
//       RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] UR3e reached idx=%zu", this->ts().c_str(), sp.ur3_idx);
//       this->run_after_hook_barrier_only(/*is_left=*/true, just_finished_idx);
//       if (idx_rb3_ >= sp.rb3_idx) sp.reached_rb3 = true;  // 이미 ≥면 도달
//     }

//     if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
//       sp.fired = true;
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

// bool MoveSequenceClient::try_fire_next_sync_after_inflight_pair(size_t just_rb3_idx, size_t just_ur3_idx)
// {
//   for (auto &sp : sync_points_) {
//     if (sp.rb3_idx == just_rb3_idx && sp.ur3_idx == just_ur3_idx) {
//       if (sp.fired) return false;
//       sp.fired = true;

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

//   if (rb3_idx == 4 && ur3_idx == 2) {
//     hold_until_degcheck_ = true;
//     sync_inflight_.reset();
//     RCLCPP_INFO(this->get_logger(), "[%s] [DEGCHECK] Holding at (RB3=4, UR3=2). Call `/zmk_DegcheckFlag`.", this->ts().c_str());
//     return;
//   }

//   if (try_fire_next_sync_after_inflight_pair(rb3_idx, ur3_idx)) {
//     return;
//   }

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
//     if (auto_restart_once_) {
//       RCLCPP_INFO(this->get_logger(),
//         "[%s] One-shot sequence finished → RESTORE MAIN & restart from step 0.", this->ts().c_str());
//       auto_restart_once_ = false;
//       restore_base_sequences_hooks_sync();
//       this->reset_sequence_state();
//       this->start_sequence();
//       return;
//     }
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
//                     action == GripperAction::OPEN  ? "OPEN"  :
//                     action == GripperAction::PINCH ? "PINCH" : "HOME");

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







//---------------rb3 step4에서 tcp로 내가 준 각도만큼 돌아감--------------

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

// #include "math.h"

// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/parameter_client.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <action_msgs/msg/goal_status_array.hpp>
// #include <action_msgs/srv/cancel_goal.hpp>

// // [NEW] 첨부 서비스 헤더 (패키지명은 실제 환경에 맞게 조정)
// #include "dual_arm_msg/srv/rotblock.hpp"   // ← 필요 시 패키지명 변경
// using RotblockSrv = dual_arm_msg::srv::Rotblock;

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

//   // [NEW] Rotblock 서비스 서버 핸들
//   rclcpp::Service<RotblockSrv>::SharedPtr srv_rotblock_;

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

//   // base snapshot (main)
//   std::vector<Pose6> base_seq_rb3_, base_seq_ur3_;
//   std::unordered_map<size_t, GripperAction> base_ur3_grip_before_;
//   std::unordered_map<size_t, GripperAction> base_ur3_grip_after_;
//   std::unordered_map<size_t, GripperAction> base_rb3_grip_before_;
//   std::unordered_map<size_t, GripperAction> base_rb3_grip_after_;
//   std::vector<PairSync> base_sync_points_;

//   // soft-restart flags
//   std::atomic<bool> restart_pending_{false};
//   bool block_progress_{false};
//   rclcpp::TimerBase::SharedPtr restart_watchdog_;

//   rclcpp::TimerBase::SharedPtr apply_wait_timer_;

//   bool auto_restart_once_ = false;

//   // ---------- helpers (decl) ----------
//   std::string ts() const;
//   static bool ends_with(const std::string &s, const std::string &suffix);
//   static bool any_active(const GoalStatusArray &arr);
//   std::string resolve_status_topic(const std::string &base);

//   std::string controller_ns_from_fjt_base(const std::string &fjt_base);
//   void configure_soft_stop(double seconds);

//   void start_sequence();
//   void reset_sequence_state();
//   bool cancel_all_goals();
//   bool cancel_right_only();

//   void snapshot_base_sequences_hooks_sync();
//   void restore_base_sequences_hooks_sync();

//   void sync_reconcile_with_current_indices();

//   void load_sequence_HOME();

//   void prepare_and_fire_first_pair();
//   void fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks);
//   void fire_both_immediate(size_t rb3_idx, size_t ur3_idx);
//   void fire_rb3_only_now(size_t rb3_idx);

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

//   void request_soft_restart();
//   void maybe_restart_after_idle();
//   void do_restart_now();

//   // [NEW] 유틸/헬퍼 (각도 변환 & 임의 포즈 전송)
//   static inline double deg2rad(double d) { return d * M_PI / 180.0; }
//   static inline double rad2deg(double r) { return r * 180.0 / M_PI; }
//   void send_rb3_pose(const Pose6 &p);  // 플랜지 회전용 단독 발사
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

//   // soft stop 0.5s
//   configure_soft_stop(0.5);

//   // ---- main sequences/hooks/sync ----
//   seq_rb3_ = {
//     {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},   // (0)
//     {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245}, // (1)
//     {-0.04642, -0.44152, 0.120, 1.76208441,    0.24801129,  1.77238186}, // (2)
//     { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},  // (3)
//     { 0.07944, -0.22198, 0.41277, 1.84917634, -1.4379768,   1.4158111},  // (4)
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

//   ur3_grip_before_.clear();
//   ur3_grip_after_.clear();
//   rb3_grip_before_.clear();
//   rb3_grip_after_.clear();
//   ur3_grip_before_[0] = GripperAction::OPEN;
//   ur3_grip_after_[1]  = GripperAction::CLOSE;
//   ur3_grip_after_[4]  = GripperAction::OPEN;

//   rb3_grip_after_[0]  = GripperAction::OPEN;
//   rb3_grip_before_[2] = GripperAction::PINCH;
//   rb3_grip_after_[2]  = GripperAction::CLOSE;
//   rb3_grip_before_[3] = GripperAction::CLOSE;
//   rb3_grip_after_[8]  = GripperAction::PINCH;
//   rb3_grip_after_[9]  = GripperAction::HOME;

//   // 동기점 유지
//   sync_points_.clear();
//   sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/2});
//   sync_points_.push_back(PairSync{/*rb3_idx=*/5, /*ur3_idx=*/3, /*rb3_next=*/6, /*ur3_next=*/4});
  
//   snapshot_base_sequences_hooks_sync();

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
//       RCLCPP_INFO(this->get_logger(), "[%s] Degcheck release → (RB3=5, UR3=3)", this->ts().c_str());
//       idx_rb3_ = 5; idx_ur3_ = 3;
//       sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;
//       this->fire_both_immediate(idx_rb3_, idx_ur3_);
//       resp->success = true;
//       resp->message = "Proceed fired to (RB3=5, UR3=3).";
//     }
//   );
//   RCLCPP_INFO(get_logger(), "Service server ready: /zmk_DegcheckFlag");

//   // ★ pickblock: 조건 분기
//   srv_pickblock_ = this->create_service<std_srvs::srv::Trigger>(
//     "zmk_pickblockchkFlag",
//     [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
//           std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
//     {
//       if (idx_rb3_ >= 6) {
//         block_progress_ = true;
//         apply_wait_timer_ = this->create_wall_timer(
//           100ms,
//           [this]() {
//             if (!rb3_waiting_ && !ur3_waiting_) {
//               if (apply_wait_timer_) apply_wait_timer_->cancel();
//               load_sequence_HOME();
//               auto_restart_once_ = false;
//               exit_when_done_ = true;
//               reset_sequence_state();
//               block_progress_ = false;
//               start_sequence();
//             }
//           }
//         );
//         resp->success = true;
//         resp->message = "HOME sequence (both) will run once, then exit.";
//         return;
//       }

//       block_progress_ = true;
//       apply_wait_timer_ = this->create_wall_timer(
//         100ms,
//         [this]() {
//           if (!rb3_waiting_) {
//             if (apply_wait_timer_) apply_wait_timer_->cancel();
//             for (auto &sp : sync_points_) { sp.reached_rb3=false; sp.reached_ur3=false; sp.fired=false; }
//             sync_inflight_.reset();
//             rb3_inflight_done_ = ur3_inflight_done_ = false;
//             hold_until_degcheck_ = false;

//             rb3_done_all_ = false;
//             rb3_seen_active_ = false;

//             size_t start_from = (seq_rb3_.size() > 1 ? 1 : 0);
//             idx_rb3_ = start_from;

//             sync_reconcile_with_current_indices();

//             block_progress_ = false;

//             start_rb3_step(start_from);
//             if (!ur3_waiting_) start_ur3_step(idx_ur3_);
//           }
//         }
//       );

//       resp->success = true;
//       resp->message = "Partial restart: RB3 from 0, UR3 keeps current step.";
//     }
//   );

//   RCLCPP_INFO(get_logger(), "Service server ready: /zmk_pickblockchkFlag");

//   // // [NEW] zmk_rotblock: Degcheck 홀드에서 RB3 플랜지 각도만 회전
//   // srv_rotblock_ = this->create_service<RotblockSrv>(
//   //   "zmk_rotblock",
//   //   [this](const std::shared_ptr<RotblockSrv::Request> req,
//   //          std::shared_ptr<RotblockSrv::Response> resp)
//   //   {
//   //     // Degcheck 홀드 구간에서만 허용
//   //     if (!hold_until_degcheck_ || idx_rb3_ != 4) {
//   //       resp->success = false;
//   //       resp->message = "Not in Degcheck hold at RB3=4 (UR3=2).";
//   //       return;
//   //     }

//   //     // 현재 타겟 포즈 읽기 (RB3 step 4 기준)
//   //     Pose6 p = seq_rb3_.at(4);
//   //     const double cur_deg = rad2deg(p.rz);
//   //     const double tgt_deg = req->angle_deg;
//   //     p.rz = deg2rad(tgt_deg);       // 플랜지(RZ)만 변경

//   //     // 단독 RB3 호출 (UR3는 유지)
//   //     send_rb3_pose(p);

//   //     resp->success = true;
//   //     std::ostringstream oss;
//   //     oss << "RB3 flange rotated: " << std::fixed << std::setprecision(1)
//   //         << cur_deg << "° → " << tgt_deg << "°";
//   //     resp->message = oss.str();
//   //   }
//   // );

//   srv_rotblock_ = this->create_service<RotblockSrv>(
//     "zmk_rotblock",
//     [this](const std::shared_ptr<RotblockSrv::Request> req,
//           std::shared_ptr<RotblockSrv::Response> resp)
//     {
//       // Degcheck 홀드 구간에서만 허용
//       if (!hold_until_degcheck_ || idx_rb3_ != 4) {
//         resp->success = false;
//         resp->message = "Not in Degcheck hold at RB3=4 (UR3=2).";
//         return;
//       }

//       // 현재 타겟 포즈 읽기 (RB3 step 4 기준)
//       Pose6 p = seq_rb3_.at(4);
//       RCLCPP_INFO(
//         this->get_logger(),
//         "[%s] RB3 step4 pose → x=%.5f, y=%.5f, z=%.5f, rx=%.5f, ry=%.5f, rz=%.5f",
//         this->ts().c_str(),
//         p.x, p.y, p.z, p.rx, p.ry, p.rz
//       );

//       // --- RY에 Δdeg를 더하는 상대 회전 ---
//       const double cur_deg   = rad2deg(p.ry);        // 현재 RY(도)
//       const double delta_deg = req->angle_deg;       // 증분(도)
//       double tgt_deg         = cur_deg + delta_deg;  // 목표(도)

//       // 라디안으로 적용
//       auto norm_pi = [](double a){
//         // [-pi, pi] 정규화 (필요 없으면 이 블록 제거 가능)
//         while (a >  M_PI) a -= 2.0*M_PI;
//         while (a <= -M_PI) a += 2.0*M_PI;
//         return a;
//       };
//       p.ry = norm_pi(deg2rad(tgt_deg));

//       // 단독 RB3 호출 (UR3는 유지, 시퀀스 인덱스/동기 미변경)
//       send_rb3_pose(p);

//       // 응답
//       resp->success = true;
//       std::ostringstream oss;
//       oss << std::fixed << std::setprecision(1)
//           << "RB3 flange RY rotated: " << cur_deg << "° → " << tgt_deg
//           << "° (Δ=" << delta_deg << "°)";
//       resp->message = oss.str();
//     }
//   );

//   RCLCPP_INFO(get_logger(), "Service server ready: /zmk_rotblock");

//   start_timer_ = this->create_wall_timer(300ms, [this] {
//     if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s) ||
//         !cli_grip_open_->wait_for_service(0s) || !cli_grip_close_->wait_for_service(0s) ||
//         !cli_hand_setangle_->wait_for_service(0s) ||
//         (use_bimanual_srv_ && !client_bimanual_tcppos_->wait_for_service(0s))) {
//       RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//         "[%s] Waiting for services (rb3/ur3e/gripper/hand%s). hand=%s, bimanual=%s",
//         this->ts().c_str(), use_bimanual_srv_ ? "/bimanual" : "",
//         hand_service_name_.c_str(), bimanual_srv_name_.c_str());
//       return;
//     }
//     const auto right_status = this->resolve_status_topic(right_fjt_base_);
//     const auto left_status  = this->resolve_status_topic(left_fjt_base_);
//     if (right_status.empty() || left_status.empty()) {
//       RCLCPP_ERROR(this->get_logger(),
//         "[%s] Could not resolve action status topics. right=%s left=%s",
//         this->ts().c_str(), right_fjt_base_.c_str(), left_fjt_base_.c_str());
//       return;
//     }
//     RCLCPP_INFO(this->get_logger(), "[%s] [RESOLVED] RIGHT: %s", this->ts().c_str(), right_status.c_str());
//     RCLCPP_INFO(this->get_logger(), "[%s] [RESOLVED] LEFT : %s", this->ts().c_str(), left_status.c_str());

//     sub_right_status_ = this->create_subscription<GoalStatusArray>(
//       right_status, rclcpp::QoS(50),
//       std::bind(&MoveSequenceClient::on_right_status, this, std::placeholders::_1));
//     sub_left_status_ = this->create_subscription<GoalStatusArray>(
//       left_status, rclcpp::QoS(50),
//       std::bind(&MoveSequenceClient::on_left_status, this, std::placeholders::_1));

//     RCLCPP_INFO(this->get_logger(), "[%s] Services ready. hand=%s, bimanual=%s(use=%s)",
//                 this->ts().c_str(), hand_service_name_.c_str(), bimanual_srv_name_.c_str(),
//                 use_bimanual_srv_ ? "true" : "false");

//     start_timer_->cancel();

//     // 시작 직전 동기점 선반영
//     sync_reconcile_with_current_indices();
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

// // controller ns from fjt base
// std::string MoveSequenceClient::controller_ns_from_fjt_base(const std::string &fjt_base)
// {
//   const std::string suf = "/follow_joint_trajectory";
//   if (ends_with(fjt_base, suf)) {
//     return fjt_base.substr(0, fjt_base.size() - suf.size());
//   }
//   auto pos = fjt_base.rfind('/');
//   if (pos == std::string::npos) return fjt_base;
//   return fjt_base.substr(0, pos);
// }

// void MoveSequenceClient::configure_soft_stop(double seconds)
// {
//   auto set_for = [this, seconds](const std::string& fjt_base) {
//     rclcpp::SyncParametersClient client(this, controller_ns_from_fjt_base(fjt_base));
//     if (!client.wait_for_service(std::chrono::seconds(1))) {
//       RCLCPP_WARN(this->get_logger(), "[%s] Soft-stop param server not available for %s", this->ts().c_str(), fjt_base.c_str());
//       return;
//     }
//     try {
//       auto results = client.set_parameters({ rclcpp::Parameter("stop_trajectory_duration", seconds) });
//       bool ok = results.empty() ? false : results.front().successful;
//       if (ok) {
//         RCLCPP_INFO(this->get_logger(), "[%s] Soft-stop set: %s=%.3f s", this->ts().c_str(), "stop_trajectory_duration", seconds);
//       } else {
//         RCLCPP_WARN(this->get_logger(), "[%s] Soft-stop set failed for %s", this->ts().c_str(), fjt_base.c_str());
//       }
//     } catch (const std::exception& e) {
//       RCLCPP_WARN(this->get_logger(), "[%s] Soft-stop set threw for %s: %s", this->ts().c_str(), fjt_base.c_str(), e.what());
//     }
//   };
//   set_for(right_fjt_base_);
//   set_for(left_fjt_base_);
// }

// // ---- base snapshot/restore ----
// void MoveSequenceClient::snapshot_base_sequences_hooks_sync()
// {
//   base_seq_rb3_ = seq_rb3_;
//   base_seq_ur3_ = seq_ur3_;
//   base_ur3_grip_before_ = ur3_grip_before_;
//   base_ur3_grip_after_  = ur3_grip_after_;
//   base_rb3_grip_before_ = rb3_grip_before_;
//   base_rb3_grip_after_  = rb3_grip_after_;
//   base_sync_points_     = sync_points_;
//   RCLCPP_INFO(this->get_logger(), "[%s] [BASE] snapshot stored.", this->ts().c_str());
// }

// void MoveSequenceClient::restore_base_sequences_hooks_sync()
// {
//   seq_rb3_ = base_seq_rb3_;
//   seq_ur3_ = base_seq_ur3_;
//   ur3_grip_before_ = base_ur3_grip_before_;
//   ur3_grip_after_  = base_ur3_grip_after_;
//   rb3_grip_before_ = base_rb3_grip_before_;
//   rb3_grip_after_  = base_rb3_grip_after_;
//   sync_points_     = base_sync_points_;
//   RCLCPP_INFO(this->get_logger(), "[%s] [BASE] restored MAIN sequences/hooks/sync.", this->ts().c_str());
// }

// // 동기점 선반영
// void MoveSequenceClient::sync_reconcile_with_current_indices()
// {
//   for (auto &sp : sync_points_) {
//     if (sp.fired) continue;
//     if (idx_rb3_ >= sp.rb3_idx) sp.reached_rb3 = true;
//     if (idx_ur3_ >= sp.ur3_idx) sp.reached_ur3 = true;

//     if (sp.reached_rb3 && sp.reached_ur3) {
//       sp.fired = true;
//       if (restart_pending_.load() || block_progress_) {
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC-RECON] restart pending → skip fire", this->ts().c_str());
//         continue;
//       }
//       idx_rb3_ = sp.rb3_next;
//       idx_ur3_ = sp.ur3_next;

//       sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;

//       RCLCPP_INFO(this->get_logger(),
//         "[%s] [SYNC-RECON] BOTH already ≥ targets → FIRE (RB3=%zu, UR3=%zu)",
//         this->ts().c_str(), idx_rb3_, idx_ur3_);
//       this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
//       break;
//     }
//   }
// }

// // ★ HOME 시퀀스 로드 (one-shot)
// void MoveSequenceClient::load_sequence_HOME()
// {
//   Pose6 rb3_before_home = { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576};
//   Pose6 rb3_home =        {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708};
//   Pose6 ur3_before_home = {-0.13056, -0.32030, 0.46642, 2.151,       0.109,       -2.257};
//   Pose6 ur3_home =        {-0.140,   -0.410,   0.350,   0.0,         -3.14,       0.0};

//   seq_rb3_.assign({ rb3_before_home, rb3_home });
//   seq_ur3_.assign({ ur3_before_home, ur3_home });

//   ur3_grip_before_.clear();
//   ur3_grip_after_.clear();
//   rb3_grip_before_.clear();
//   rb3_grip_after_.clear();
//   sync_points_.clear();

//   RCLCPP_INFO(this->get_logger(), "[%s] HOME sequence loaded (one-shot).", this->ts().c_str());
// }

// // ---- restart/cancel ----
// void MoveSequenceClient::start_sequence() {
//   RCLCPP_INFO(this->get_logger(), "[%s] === Sequence PREP ===", this->ts().c_str());
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

// bool MoveSequenceClient::cancel_right_only()
// {
//   if (!cancel_right_cli_->wait_for_service(500ms)) {
//     RCLCPP_ERROR(this->get_logger(), "[%s] CancelGoal RIGHT service not available", this->ts().c_str());
//     return false;
//   }
//   auto req = std::make_shared<action_msgs::srv::CancelGoal::Request>();
//   for (auto &b : req->goal_info.goal_id.uuid) b = 0;
//   req->goal_info.stamp.sec = 0;
//   req->goal_info.stamp.nanosec = 0;

//   auto f_r = cancel_right_cli_->async_send_request(req);
//   bool ok_r = (f_r.wait_for(1s) == std::future_status::ready);
//   if (!ok_r) {
//     RCLCPP_ERROR(this->get_logger(), "[%s] CancelGoal RIGHT timeout", this->ts().c_str());
//     return false;
//   }
//   try {
//     auto r = f_r.get();
//     RCLCPP_INFO(this->get_logger(), "[%s] CancelGoal RIGHT ret(code=%d)", this->ts().c_str(), r->return_code);
//   } catch (...) {
//     RCLCPP_WARN(this->get_logger(), "[%s] CancelGoal RIGHT exception (ignored)", this->ts().c_str());
//   }
//   std::this_thread::sleep_for(100ms);
//   return true;
// }

// // ---- first pair ----
// void MoveSequenceClient::prepare_and_fire_first_pair() {
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

// // [NEW] RB3 단독 포즈 송신(플랜지 회전용). 인덱스/동기 advancing 없음.
// void MoveSequenceClient::send_rb3_pose(const Pose6 &p)
// {
//   auto req = std::make_shared<RB3Srv::Request>();
//   req->right_x = p.x; req->right_y = p.y; req->right_z = p.z;
//   req->right_rx = p.rx; req->right_ry = p.ry; req->right_rz = p.rz;

//   // rb3_waiting_ = true;
//   // rb3_seen_active_ = false;
//   // t_send_rb3_ = std::chrono::steady_clock::now();

//   // cli_rb3_->async_send_request(
//   //   req,
//   //   [this](rclcpp::Client<RB3Srv>::SharedFuture future) {
//   //     (void)future; // 최소 로그, 에러는 콜백 내부에서 컨트롤러가 처리
//   //   });
//   cli_rb3_->async_send_request(
//   req,
//   [this](rclcpp::Client<RB3Srv>::SharedFuture /*future*/) {
//     // 여기서 rb3_waiting_ / rb3_seen_active_ 변경 금지!
//   });
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

// void MoveSequenceClient::fire_rb3_only_now(size_t rb3_idx)
// {
//   if (restart_pending_.load()) {
//     RCLCPP_INFO(this->get_logger(), "[%s] Soft-restart pending → skip FIRE RB3 ONLY", this->ts().c_str());
//     return;
//   }

//   const auto &r = seq_rb3_[rb3_idx];

//   // RB3 대기/타임스탬프 갱신 (상태콜백에서 완료 판정에 필요)
//   rb3_waiting_     = true;
//   rb3_seen_active_ = false;
//   t_send_rb3_      = std::chrono::steady_clock::now();

//   auto req = std::make_shared<RB3Srv::Request>();
//   req->right_x  = r.x;  req->right_y  = r.y;  req->right_z  = r.z;
//   req->right_rx = r.rx; req->right_ry = r.ry; req->right_rz = r.rz;

//   RCLCPP_INFO(this->get_logger(), "[%s] FIRE RB3 ONLY → step %zu", this->ts().c_str(), rb3_idx);

//   cli_rb3_->async_send_request(
//     req,
//     [this, rb3_idx](rclcpp::Client<RB3Srv>::SharedFuture f) {
//       bool ok = false;
//       try { auto resp = f.get(); ok = resp && resp->success; } catch (...) {}
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB3 ONLY] step %zu request %s",
//                   this->ts().c_str(), rb3_idx, ok ? "OK" : "FAILED");
//     });
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
//       if (idx_ur3_ >= sp.ur3_idx) sp.reached_ur3 = true;
//     }
//     if ( is_left && just_finished_idx == sp.ur3_idx) {
//       sp.reached_ur3 = true; matched_any = true;
//       RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] UR3e reached idx=%zu", this->ts().c_str(), sp.ur3_idx);
//       this->run_after_hook_barrier_only(/*is_left=*/true, just_finished_idx);
//       if (idx_rb3_ >= sp.rb3_idx) sp.reached_rb3 = true;
//     }

//     if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
//       sp.fired = true;
//       if (restart_pending_.load() || block_progress_) {
//         RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] Soft-restart pending → not firing next pair", this->ts().c_str());
//         return true;
//       }

//       const size_t next_rb3 = sp.rb3_next; // 4
//       const size_t next_ur3 = sp.ur3_next; // 2

//       if (sp.rb3_idx == 3 && sp.ur3_idx == 2) {
//         // RB3만 4로: UR3 인덱스는 '되돌리지 않음'
//         idx_rb3_ = next_rb3;
//         // idx_ur3_ = std::max(idx_ur3_, next_ur3);  // 필요시만

//         sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//         rb3_inflight_done_ = ur3_inflight_done_ = false;

//         RCLCPP_INFO(this->get_logger(),
//           "[%s] [SYNC] (3,2) → RB3 ONLY FIRE to 4 (UR3 stays at %zu)",
//           this->ts().c_str(), idx_ur3_);

//         this->fire_rb3_only_now(idx_rb3_);
//         return true;
//       }

//       // 그 외 동기는 기존대로 동시 발사
//       idx_rb3_ = next_rb3;
//       idx_ur3_ = next_ur3;
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

// bool MoveSequenceClient::try_fire_next_sync_after_inflight_pair(size_t just_rb3_idx, size_t just_ur3_idx)
// {
//   for (auto &sp : sync_points_) {
//     if (sp.rb3_idx == just_rb3_idx && sp.ur3_idx == just_ur3_idx) {
//       if (sp.fired) return false;
//       sp.fired = true;

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

//   if (rb3_idx == 4 && ur3_idx == 2) {
//     hold_until_degcheck_ = true;
//     sync_inflight_.reset();
//     RCLCPP_INFO(this->get_logger(), "[%s] [DEGCHECK] Holding at (RB3=4, UR3=2). Call `/zmk_DegcheckFlag`.", this->ts().c_str());
//     return;
//   }

//   if (try_fire_next_sync_after_inflight_pair(rb3_idx, ur3_idx)) {
//     return;
//   }

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
//     if (auto_restart_once_) {
//       RCLCPP_INFO(this->get_logger(),
//         "[%s] One-shot sequence finished → RESTORE MAIN & restart from step 0.", this->ts().c_str());
//       auto_restart_once_ = false;
//       restore_base_sequences_hooks_sync();
//       this->reset_sequence_state();
//       this->start_sequence();
//       return;
//     }
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
//   }
//   auto t0 = std::chrono::steady_clock::now();
//   cli->async_send_request(
//     req,
//     [this, action, on_done, t0](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
//       bool ok = false;
//       try { auto resp = future.get(); ok = resp && resp->success; } catch (...) {}
//       auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
//                   std::chrono::steady_clock::now() - t0).count();
//       RCLCPP_INFO(this->get_logger(), "[%s] [UR %s] %s (Δ=%lldms)",
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
//                     action == GripperAction::OPEN  ? "OPEN"  :
//                     action == GripperAction::PINCH ? "PINCH" : "HOME");

//   auto t0 = std::chrono::steady_clock::now();
//   cli_hand_setangle_->async_send_request(
//     req,
//     [this, on_done, t0, tag](rclcpp::Client<HandSetAngleSrv>::SharedFuture future) {
//       bool ok = false;
//       try { auto resp = future.get(); ok = (resp != nullptr); } catch (...) {}
//       auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
//                   std::chrono::steady_clock::now() - t0).count();
//       RCLCPP_INFO(this->get_logger(), "[%s] [RB HAND %s] %s (Δ=%lldms)",
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
//       maybe_restart_after_idle();
//     }
//   }
// }

// // ---- soft-restart helpers (IMPLEMENTATION) ----
// void MoveSequenceClient::request_soft_restart()
// {
//   restart_pending_.store(true);
//   block_progress_ = true;

//   if (!restart_watchdog_ || restart_watchdog_->is_canceled()) {
//     restart_watchdog_ = this->create_wall_timer(
//       100ms, [this]() {
//         if (!restart_pending_.load()) { restart_watchdog_->cancel(); return; }
//         if (!rb3_waiting_ && !ur3_waiting_) {
//           restart_watchdog_->cancel();
//           do_restart_now();
//         }
//       }
//     );
//   }
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
//   this->reset_sequence_state();
//   restart_pending_.store(false);
//   block_progress_ = false;
//   this->start_sequence();
// }

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MoveSequenceClient>());
//   rclcpp::shutdown();
//   return 0;
// }






















//-----20251016_zmk_pickblockchkFlag 서비스주면 나머지 서비스 동작 안함----------

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

#include "math.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

// TF2 (Humble)
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// [NEW] 첨부 서비스 헤더 (패키지명은 실제 환경에 맞게 조정)
#include "dual_arm_msg/srv/rotblock.hpp"   // ← 필요 시 패키지명 변경
using RotblockSrv = dual_arm_msg::srv::Rotblock;

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

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_pickblock_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_degcheck_;

  // [NEW] Rotblock 서비스 서버 핸들
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

  rclcpp::TimerBase::SharedPtr sync_fire_timer_;

  bool hold_until_degcheck_{false};

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

  // soft-restart flags
  std::atomic<bool> restart_pending_{false};
  bool block_progress_{false};
  rclcpp::TimerBase::SharedPtr restart_watchdog_;

  rclcpp::TimerBase::SharedPtr apply_wait_timer_;

  rclcpp::TimerBase::SharedPtr restart_delay_timer_;

  bool auto_restart_once_ = false;

  // ---------- TF (for current TCP read) ----------
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string rb3_base_frame_;
  std::string rb3_tcp_frame_;

  // ---------- helpers (decl) ----------
  std::string ts() const;
  static bool ends_with(const std::string &s, const std::string &suffix);
  static bool any_active(const GoalStatusArray &arr);
  std::string resolve_status_topic(const std::string &base);

  std::string controller_ns_from_fjt_base(const std::string &fjt_base);
  void configure_soft_stop(double seconds);

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

  // [NEW] 유틸/헬퍼 (각도 변환 & 임의 포즈 전송)
  static inline double deg2rad(double d) { return d * M_PI / 180.0; }
  static inline double rad2deg(double r) { return r * 180.0 / M_PI; }
  void send_rb3_pose(const Pose6 &p);  // 플랜지 회전용 단독 발사
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

  // [NEW] TF frame parameters
  rb3_base_frame_ = declare_parameter<std::string>("rb3_base_frame", "link0");
  rb3_tcp_frame_  = declare_parameter<std::string>("rb3_tcp_frame",  "tcp");

  // [NEW] TF buffer/listener
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

  // soft stop 0.5s
  configure_soft_stop(0.5);

  // ---- main sequences/hooks/sync ----
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

  // 동기점 유지
  sync_points_.clear();
  sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/2});
  sync_points_.push_back(PairSync{/*rb3_idx=*/5, /*ur3_idx=*/3, /*rb3_next=*/6, /*ur3_next=*/4});
  
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
      RCLCPP_INFO(this->get_logger(), "[%s] Degcheck release → (RB3=5, UR3=3)", this->ts().c_str());
      idx_rb3_ = 5; idx_ur3_ = 3;
      sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
      rb3_inflight_done_ = ur3_inflight_done_ = false;
      this->fire_both_immediate(idx_rb3_, idx_ur3_);
      resp->success = true;
      resp->message = "Proceed fired to (RB3=5, UR3=3).";
    }
  );
  RCLCPP_INFO(get_logger(), "Service server ready: /zmk_DegcheckFlag");

  // ★ pickblock: 조건 분기
  srv_pickblock_ = this->create_service<std_srvs::srv::Trigger>(
    "zmk_pickblockchkFlag",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
          std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
    {
      if (idx_rb3_ >= 6) {
        block_progress_ = true;
        apply_wait_timer_ = this->create_wall_timer(
          100ms,
          [this]() {
            if (!rb3_waiting_ && !ur3_waiting_) {
              if (apply_wait_timer_) apply_wait_timer_->cancel();
              load_sequence_HOME();
              auto_restart_once_ = false;
              exit_when_done_ = true;
              reset_sequence_state();
              block_progress_ = false;
              start_sequence();
            }
          }
        );
        resp->success = true;
        resp->message = "HOME sequence (both) will run once, then exit.";
        return;
      }

      block_progress_ = true;
      apply_wait_timer_ = this->create_wall_timer(
        100ms,
        [this]() {
          if (!rb3_waiting_) {
            if (apply_wait_timer_) apply_wait_timer_->cancel();
            for (auto &sp : sync_points_) { sp.reached_rb3=false; sp.reached_ur3=false; sp.fired=false; }
            sync_inflight_.reset();
            rb3_inflight_done_ = ur3_inflight_done_ = false;
            hold_until_degcheck_ = false;

            rb3_done_all_ = false;
            rb3_seen_active_ = false;

            size_t start_from = (seq_rb3_.size() > 2 ? 2 : 0);
            idx_rb3_ = start_from;

            sync_reconcile_with_current_indices();

            Pose6 drop_pose = { -0.0407, -0.44568, 0.450,
                                1.7228145, 0.17715092, 1.80205245 };
            RCLCPP_INFO(this->get_logger(),
              "[%s] [RESTART] move RB3 to drop pose (idx1) then wait 5s", this->ts().c_str());
            this->send_rb3_pose(drop_pose);

            // ★ 여기서 바로 block_progress_ 해제/재시작하지 말고 5초 대기 후 시작
            const auto delay = std::chrono::seconds(5);
            RCLCPP_INFO(this->get_logger(),
              "[%s] [RESTART] main step<6 → delaying %.1fs before resume",
              this->ts().c_str(), (double)delay.count());

            restart_delay_timer_ = this->create_wall_timer(
              delay,
              [this, start_from]() {
                if (restart_delay_timer_) restart_delay_timer_->cancel();
                // 지연 끝난 뒤에 해제하고 실제 재시작
                block_progress_ = false;
                RCLCPP_INFO(this->get_logger(), "[%s] [RESTART] delay done → resume", this->ts().c_str());
                this->start_rb3_step(start_from);
                if (!ur3_waiting_) this->start_ur3_step(idx_ur3_);
              }
            );
            return;

            // block_progress_ = false;

            // start_rb3_step(start_from);
            // if (!ur3_waiting_) start_ur3_step(idx_ur3_);
          }
        }
      );

      resp->success = true;
      resp->message = "Partial restart: RB3 from 0, UR3 keeps current step.";
    }
  );

  RCLCPP_INFO(get_logger(), "Service server ready: /zmk_pickblockchkFlag");

  srv_rotblock_ = this->create_service<RotblockSrv>(
    "zmk_rotblock",
    [this](const std::shared_ptr<RotblockSrv::Request> req,
          std::shared_ptr<RotblockSrv::Response> resp)
    {
      // Degcheck 홀드 구간에서만 허용
      if (!hold_until_degcheck_ || idx_rb3_ != 4) {
        resp->success = false;
        resp->message = "Not in Degcheck hold at RB3=4 (UR3=2).";
        return;
      }

      Pose6 p{};
      bool used_tf = false;

      // 현재 자세 원본(RPY) 저장용
      double base_rx = 0.0, base_ry = 0.0, base_rz = 0.0;

      // --- TF로 현재 TCP 읽기 ---
      try {
        const auto tf = tf_buffer_->lookupTransform(
          rb3_base_frame_, rb3_tcp_frame_, tf2::TimePointZero);

        // 위치
        p.x = tf.transform.translation.x;
        p.y = tf.transform.translation.y;
        p.z = tf.transform.translation.z;

        // 오리엔테이션(quat → RPY)
        const auto &q = tf.transform.rotation;
        tf2::Quaternion q_cur(q.x, q.y, q.z, q.w);
        q_cur.normalize();

        tf2::Matrix3x3(q_cur).getRPY(base_rx, base_ry, base_rz);
        p.rx = base_rx; p.ry = base_ry; p.rz = base_rz;

        RCLCPP_INFO(
          this->get_logger(),
          "[%s] TF(%s→%s) TCP → x=%.5f y=%.5f z=%.5f | RPY(rad)=%.6f %.6f %.6f | (deg)=%.2f %.2f %.2f",
          this->ts().c_str(),
          rb3_base_frame_.c_str(), rb3_tcp_frame_.c_str(),
          p.x, p.y, p.z,
          p.rx, p.ry, p.rz,
          p.rx*180.0/M_PI, p.ry*180.0/M_PI, p.rz*180.0/M_PI
        );

        used_tf = true;

        // ---- Δ회전(로컬 축 기준) 합성: q_new = q_cur * dQx * dQy * dQz ----
        const double d_rx = deg2rad(req->r_rx_deg);
        const double d_ry = deg2rad(req->r_ry_deg);
        const double d_rz = deg2rad(req->r_rz_deg);

        tf2::Quaternion dQx, dQy, dQz;
        dQx.setRPY(d_rx, 0, 0);
        dQy.setRPY(0, d_ry, 0);
        dQz.setRPY(0, 0, d_rz);

        tf2::Quaternion q_new = q_cur * dQx * dQy * dQz;  // 로컬(TCP) 축 기준
        q_new.normalize();

        double roll=0, pitch=0, yaw=0;
        tf2::Matrix3x3(q_new).getRPY(roll, pitch, yaw);
        p.rx = roll; p.ry = pitch; p.rz = yaw;

      } catch (const tf2::TransformException &ex) {
        // 폴백: 기존 step4 타겟
        p = seq_rb3_.at(4);
        base_rx = p.rx; base_ry = p.ry; base_rz = p.rz;

        RCLCPP_WARN(
          this->get_logger(),
          "[%s] TF lookup failed (%s→%s): %s → fallback to seq_rb3_[4]",
          this->ts().c_str(),
          rb3_base_frame_.c_str(), rb3_tcp_frame_.c_str(), ex.what()
        );

        // 폴백에도 동일한 Δ회전 적용 (현재 p의 RPY를 쿼터니언으로 변환 후 합성)
        tf2::Quaternion q_cur;
        q_cur.setRPY(base_rx, base_ry, base_rz);
        q_cur.normalize();

        const double d_rx = deg2rad(req->r_rx_deg);
        const double d_ry = deg2rad(req->r_ry_deg);
        const double d_rz = deg2rad(req->r_rz_deg);

        tf2::Quaternion dQx, dQy, dQz;
        dQx.setRPY(d_rx, 0, 0);
        dQy.setRPY(0, d_ry, 0);
        dQz.setRPY(0, 0, d_rz);

        tf2::Quaternion q_new = q_cur * dQx * dQy * dQz;
        q_new.normalize();

        double roll=0, pitch=0, yaw=0;
        tf2::Matrix3x3(q_new).getRPY(roll, pitch, yaw);
        p.rx = roll; p.ry = pitch; p.rz = yaw;
      }

      // ---- 로그: before → after (deg) ----
      const double cur_rx_deg = rad2deg(base_rx);
      const double cur_ry_deg = rad2deg(base_ry);
      const double cur_rz_deg = rad2deg(base_rz);
      const double tgt_rx_deg = rad2deg(p.rx);
      const double tgt_ry_deg = rad2deg(p.ry);
      const double tgt_rz_deg = rad2deg(p.rz);

      RCLCPP_INFO(
        this->get_logger(),
        "[%s] Δapply (local) — RX: %.2f° → %.2f° (Δ=%.2f°), RY: %.2f° → %.2f° (Δ=%.2f°), RZ: %.2f° → %.2f° (Δ=%.2f°)",
        this->ts().c_str(),
        cur_rx_deg, tgt_rx_deg, tgt_rx_deg - cur_rx_deg,
        cur_ry_deg, tgt_ry_deg, tgt_ry_deg - cur_ry_deg,
        cur_rz_deg, tgt_rz_deg, tgt_rz_deg - cur_rz_deg
      );

      // ---- 송신 (UR3는 유지, 시퀀스 인덱스/동기 미변경) ----
      send_rb3_pose(p);

      // 응답
      resp->success = true;
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1)
          << (used_tf ? "[TF] " : "[SEQ] ")
          << "RB3 RPY rotated — "
          << "RX: " << cur_rx_deg << "° → " << tgt_rx_deg << "° (Δ=" << (tgt_rx_deg - cur_rx_deg) << "°), "
          << "RY: " << cur_ry_deg << "° → " << tgt_ry_deg << "° (Δ=" << (tgt_ry_deg - cur_ry_deg) << "°), "
          << "RZ: " << cur_rz_deg << "° → " << tgt_rz_deg << "° (Δ=" << (tgt_rz_deg - cur_rz_deg) << "°)";
      resp->message = oss.str();
    }
  );
  RCLCPP_INFO(get_logger(), "Service server ready: /zmk_rotblock");

  start_timer_ = this->create_wall_timer(300ms, [this] {
    if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s) ||
        !cli_grip_open_->wait_for_service(0s) || !cli_grip_close_->wait_for_service(0s) ||
        !cli_hand_setangle_->wait_for_service(0s) ||
        (use_bimanual_srv_ && !client_bimanual_tcppos_->wait_for_service(0s))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[%s] Waiting for services (rb3/ur3e/gripper/hand%s). hand=%s, bimanual=%s",
        this->ts().c_str(), use_bimanual_srv_ ? "/bimanual" : "",
        hand_service_name_.c_str(), bimanual_srv_name_.c_str());
      return;
    }
    const auto right_status = this->resolve_status_topic(right_fjt_base_);
    const auto left_status  = this->resolve_status_topic(left_fjt_base_);
    if (right_status.empty() || left_status.empty()) {
      RCLCPP_ERROR(this->get_logger(),
        "[%s] Could not resolve action status topics. right=%s left=%s",
        this->ts().c_str(), right_fjt_base_.c_str(), left_fjt_base_.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[%s] [RESOLVED] RIGHT: %s", this->ts().c_str(), right_status.c_str());
    RCLCPP_INFO(this->get_logger(), "[%s] [RESOLVED] LEFT : %s", this->ts().c_str(), left_status.c_str());

    sub_right_status_ = this->create_subscription<GoalStatusArray>(
      right_status, rclcpp::QoS(50),
      std::bind(&MoveSequenceClient::on_right_status, this, std::placeholders::_1));
    sub_left_status_ = this->create_subscription<GoalStatusArray>(
      left_status, rclcpp::QoS(50),
      std::bind(&MoveSequenceClient::on_left_status, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[%s] Services ready. hand=%s, bimanual=%s(use=%s)",
                this->ts().c_str(), hand_service_name_.c_str(), bimanual_srv_name_.c_str(),
                use_bimanual_srv_ ? "true" : "false");

    start_timer_->cancel();

    // 시작 직전 동기점 선반영
    sync_reconcile_with_current_indices();
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

// controller ns from fjt base
std::string MoveSequenceClient::controller_ns_from_fjt_base(const std::string &fjt_base)
{
  const std::string suf = "/follow_joint_trajectory";
  if (ends_with(fjt_base, suf)) {
    return fjt_base.substr(0, fjt_base.size() - suf.size());
  }
  auto pos = fjt_base.rfind('/');
  if (pos == std::string::npos) return fjt_base;
  return fjt_base.substr(0, pos);
}

void MoveSequenceClient::configure_soft_stop(double seconds)
{
  auto set_for = [this, seconds](const std::string& fjt_base) {
    rclcpp::SyncParametersClient client(this, controller_ns_from_fjt_base(fjt_base));
    if (!client.wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "[%s] Soft-stop param server not available for %s", this->ts().c_str(), fjt_base.c_str());
      return;
    }
    try {
      auto results = client.set_parameters({ rclcpp::Parameter("stop_trajectory_duration", seconds) });
      bool ok = results.empty() ? false : results.front().successful;
      if (ok) {
        RCLCPP_INFO(this->get_logger(), "[%s] Soft-stop set: %s=%.3f s", this->ts().c_str(), "stop_trajectory_duration", seconds);
      } else {
        RCLCPP_WARN(this->get_logger(), "[%s] Soft-stop set failed for %s", this->ts().c_str(), fjt_base.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "[%s] Soft-stop set threw for %s: %s", this->ts().c_str(), fjt_base.c_str(), e.what());
    }
  };
  set_for(right_fjt_base_);
  set_for(left_fjt_base_);
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

// 동기점 선반영
void MoveSequenceClient::sync_reconcile_with_current_indices()
{
  for (auto &sp : sync_points_) {
    if (sp.fired) continue;
    if (idx_rb3_ >= sp.rb3_idx) sp.reached_rb3 = true;
    if (idx_ur3_ >= sp.ur3_idx) sp.reached_ur3 = true;

    if (sp.reached_rb3 && sp.reached_ur3) {
      sp.fired = true;
      if (restart_pending_.load() || block_progress_) {
        RCLCPP_INFO(this->get_logger(), "[%s] [SYNC-RECON] restart pending → skip fire", this->ts().c_str());
        continue;
      }
      idx_rb3_ = sp.rb3_next;
      idx_ur3_ = sp.ur3_next;

      sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
      rb3_inflight_done_ = ur3_inflight_done_ = false;

      RCLCPP_INFO(this->get_logger(),
        "[%s] [SYNC-RECON] BOTH already ≥ targets → FIRE (RB3=%zu, UR3=%zu)",
        this->ts().c_str(), idx_rb3_, idx_ur3_);
      this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
      break;
    }
  }
}

// ★ HOME 시퀀스 로드 (one-shot)
void MoveSequenceClient::load_sequence_HOME()
{
  Pose6 rb3_before_home = { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576};
  Pose6 rb3_home =        {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708};
  Pose6 ur3_before_home = {-0.13056, -0.32030, 0.46642, 2.151,       0.109,       -2.257};
  Pose6 ur3_home =        {-0.140,   -0.410,   0.350,   0.0,         -3.14,       0.0};

  seq_rb3_.assign({ rb3_before_home, rb3_home });
  seq_ur3_.assign({ ur3_before_home, ur3_home });

  ur3_grip_before_.clear();
  ur3_grip_after_.clear();
  rb3_grip_before_.clear();
  rb3_grip_after_.clear();
  sync_points_.clear();

  RCLCPP_INFO(this->get_logger(), "[%s] HOME sequence loaded (one-shot).", this->ts().c_str());
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

bool MoveSequenceClient::cancel_right_only()
{
  if (!cancel_right_cli_->wait_for_service(500ms)) {
    RCLCPP_ERROR(this->get_logger(), "[%s] CancelGoal RIGHT service not available", this->ts().c_str());
    return false;
  }
  auto req = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  for (auto &b : req->goal_info.goal_id.uuid) b = 0;
  req->goal_info.stamp.sec = 0;
  req->goal_info.stamp.nanosec = 0;

  auto f_r = cancel_right_cli_->async_send_request(req);
  bool ok_r = (f_r.wait_for(1s) == std::future_status::ready);
  if (!ok_r) {
    RCLCPP_ERROR(this->get_logger(), "[%s] CancelGoal RIGHT timeout", this->ts().c_str());
    return false;
  }
  try {
    auto r = f_r.get();
    RCLCPP_INFO(this->get_logger(), "[%s] CancelGoal RIGHT ret(code=%d)", this->ts().c_str(), r->return_code);
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "[%s] CancelGoal RIGHT exception (ignored)", this->ts().c_str());
  }
  std::this_thread::sleep_for(100ms);
  return true;
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

// [NEW] RB3 단독 포즈 송신(플랜지 회전용). 인덱스/동기 advancing 없음.
void MoveSequenceClient::send_rb3_pose(const Pose6 &p)
{
  auto req = std::make_shared<RB3Srv::Request>();
  req->right_x = p.x; req->right_y = p.y; req->right_z = p.z;
  req->right_rx = p.rx; req->right_ry = p.ry; req->right_rz = p.rz;

  // 단독 플랜지 보정에서는 waiting 플래그를 건드리지 않음 (상태머신 보존)
  cli_rb3_->async_send_request(
    req,
    [this](rclcpp::Client<RB3Srv>::SharedFuture /*future*/) {
      // 여기서 rb3_waiting_ / rb3_seen_active_ 변경 금지!
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

void MoveSequenceClient::fire_rb3_only_now(size_t rb3_idx)
{
  if (restart_pending_.load()) {
    RCLCPP_INFO(this->get_logger(), "[%s] Soft-restart pending → skip FIRE RB3 ONLY", this->ts().c_str());
    return;
  }

  const auto &r = seq_rb3_[rb3_idx];

  // RB3 대기/타임스탬프 갱신 (상태콜백에서 완료 판정에 필요)
  rb3_waiting_     = true;
  rb3_seen_active_ = false;
  t_send_rb3_      = std::chrono::steady_clock::now();

  auto req = std::make_shared<RB3Srv::Request>();
  req->right_x  = r.x;  req->right_y  = r.y;  req->right_z  = r.z;
  req->right_rx = r.rx; req->right_ry = r.ry; req->right_rz = r.rz;

  RCLCPP_INFO(this->get_logger(), "[%s] FIRE RB3 ONLY → step %zu", this->ts().c_str(), rb3_idx);

  cli_rb3_->async_send_request(
    req,
    [this, rb3_idx](rclcpp::Client<RB3Srv>::SharedFuture f) {
      bool ok = false;
      try { auto resp = f.get(); ok = resp && resp->success; } catch (...) {}
      RCLCPP_INFO(this->get_logger(), "[%s] [RB3 ONLY] step %zu request %s",
                  this->ts().c_str(), rb3_idx, ok ? "OK" : "FAILED");
    });
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
      if (idx_ur3_ >= sp.ur3_idx) sp.reached_ur3 = true;
    }
    if ( is_left && just_finished_idx == sp.ur3_idx) {
      sp.reached_ur3 = true; matched_any = true;
      RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] UR3e reached idx=%zu", this->ts().c_str(), sp.ur3_idx);
      this->run_after_hook_barrier_only(/*is_left=*/true, just_finished_idx);
      if (idx_rb3_ >= sp.rb3_idx) sp.reached_rb3 = true;
    }

    if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
      sp.fired = true;
      if (restart_pending_.load() || block_progress_) {
        RCLCPP_INFO(this->get_logger(), "[%s] [SYNC] Soft-restart pending → not firing next pair", this->ts().c_str());
        return true;
      }

      const size_t next_rb3 = sp.rb3_next; // 4
      const size_t next_ur3 = sp.ur3_next; // 2

      if (sp.rb3_idx == 3 && sp.ur3_idx == 2) {
        // RB3만 4로: UR3 인덱스는 '되돌리지 않음'
        idx_rb3_ = next_rb3;

        sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
        rb3_inflight_done_ = ur3_inflight_done_ = false;

        RCLCPP_INFO(this->get_logger(),
          "[%s] [SYNC] (3,2) → RB3 ONLY FIRE to 4 (UR3 stays at %zu)",
          this->ts().c_str(), idx_ur3_);

        this->fire_rb3_only_now(idx_rb3_);
        return true;
      }

      // 그 외 동기는 기존대로 동시 발사
      idx_rb3_ = next_rb3;
      idx_ur3_ = next_ur3;
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
  if (!ur3_inflight_done_ && !ur3_waiting_) {
    ur3_inflight_done_ = true;
  }
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
      auto_restart_once_ = false;
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
  }
  auto t0 = std::chrono::steady_clock::now();
  cli->async_send_request(
    req,
    [this, action, on_done, t0](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      bool ok = false;
      try { auto resp = future.get(); ok = resp && resp->success; } catch (...) {}
      auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(this->get_logger(), "[%s] [UR %s] %s (Δ=%lldms)",
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
    req->angle3 = 0; req->angle4 = 300;  req->angle5 = 1000;
  }
  else if(action == GripperAction::OPEN)
  { // OPEN
    req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000;
    req->angle3 = 1000; req->angle4 = 300; req->angle5 = 1000;
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
      RCLCPP_INFO(this->get_logger(), "[%s] [RB HAND %s] %s (Δ=%lldms)",
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

// ---- soft-restart helpers (IMPLEMENTATION) ----
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

void MoveSequenceClient::do_restart_now()
{
  if (!restart_pending_.load()) return;
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













































































//----------------------tactile 감지----------------------------------------

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

// #include <std_msgs/msg/u_int16_multi_array.hpp>

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
// using UInt16MA = std_msgs::msg::UInt16MultiArray;

// struct Pose6 { double x, y, z, rx, ry, rz; };
// enum class GripperAction { NONE = 0, OPEN, CLOSE, PINCH, HOME};

// class MoveSequenceClient : public rclcpp::Node
// {
// public:
//   MoveSequenceClient();

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

//   // base snapshot
//   std::vector<Pose6> base_seq_rb3_, base_seq_ur3_;
//   std::unordered_map<size_t, GripperAction> base_ur3_grip_before_;
//   std::unordered_map<size_t, GripperAction> base_ur3_grip_after_;
//   std::unordered_map<size_t, GripperAction> base_rb3_grip_before_;
//   std::unordered_map<size_t, GripperAction> base_rb3_grip_after_;
//   std::vector<PairSync> base_sync_points_;

//   bool auto_restart_once_ = false;

//   // soft-restart flags
//   std::atomic<bool> restart_pending_{false};
//   bool block_progress_{false};
//   rclcpp::TimerBase::SharedPtr restart_watchdog_;
//   rclcpp::TimerBase::SharedPtr apply_wait_timer_;

//   // ---------- tactile (고급 판정) ----------
//   rclcpp::Subscription<UInt16MA>::SharedPtr sub_idx_tact_, sub_mid_tact_;

//   // 최근 원시 합계
//   std::atomic<int> last_idx_sum_{0}, last_mid_sum_{0};

//   // EMA
//   double ema_idx_{0.0}, ema_mid_{0.0};
//   double alpha_{0.2}; // 0.05~0.3 권장

//   // 베이스라인 자동 캘리브레이션
//   bool have_baseline_{false};
//   int baseline_samples_needed_{50};
//   int baseline_cnt_{0};
//   double baseline_acc_idx_{0.0}, baseline_acc_mid_{0.0};
//   double baseline_idx_{0.0}, baseline_mid_{0.0};

//   // 극성/임계/로깅
//   bool polarity_increase_{true}; // true: 접촉↑ / false: 접촉↓
//   int delta_idx_thr_{120};
//   int delta_mid_thr_{120};
//   int tactile_poll_ms_{100};
//   int tactile_log_every_ms_{500};
//   std::string tactile_index_topic_{"/tactile/indexnail"};
//   std::string tactile_middle_topic_{"/tactile/middlenail"};
//   rclcpp::TimerBase::SharedPtr tactile_timer_;
//   bool last_obj_present_{false};
//   std::chrono::steady_clock::time_point last_tactile_log_tp_{};
//   rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_tactile_recalib_;

//   // ---------- helpers ----------
//   std::string ts() const;
//   static bool ends_with(const std::string &s, const std::string &suffix);
//   static bool any_active(const GoalStatusArray &arr);
//   std::string resolve_status_topic(const std::string &base);

//   void start_sequence();
//   void reset_sequence_state();
//   bool cancel_all_goals();

//   void snapshot_base_sequences_hooks_sync();
//   void restore_base_sequences_hooks_sync();

//   void load_sequence_A();
//   void load_sequence_B();
//   void apply_sequence_and_soft_restart(bool use_A);

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
//   void request_soft_restart();
//   void maybe_restart_after_idle();
//   void do_restart_now();

//   // tactile helpers
//   void on_index_tactile_(const UInt16MA::SharedPtr msg);
//   void on_middle_tactile_(const UInt16MA::SharedPtr msg);
//   void tick_tactile_();
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

//   // ---- 메인 시퀀스/훅/동기점 ----
//   seq_rb3_ = {
//     {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},
//     {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245},
//     {-0.04642, -0.44152, 0.120,   1.76208441,  0.24801129,  1.77238186},
//     { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},
//     { 0.07944, -0.22198, 0.41277, 1.84917634, -1.4379768,   1.4158111},
//     { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},
//     { 0.135,   -0.36851, 0.30087, 0.1446878,   0.1537635,   1.322436},
//     { 0.050,   -0.37708, 0.31162, 0.15708,     0.2003638,   1.4264576},
//     {-0.04381, -0.45377, 0.13585, 1.75492856,  0.19338248,  1.8102555},
//     {-0.0407,  -0.44568, 0.31162, 1.7228145,   0.17715092,  1.80205245},
//     {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},
//   };
//   seq_ur3_ = {
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },
//     { -0.140,   -0.410,   0.175,   0.0,   -3.14,  0.0 },
//     { -0.0800,  -0.32281, 0.46642, 2.151,  0.109, -2.257 },
//     { -0.13056, -0.32281, 0.46642, 2.151,  0.109, -2.257 },
//     { -0.185,   -0.32281, 0.46642, 2.151,  0.109, -2.257 },
//     { -0.13056, -0.32030, 0.46642, 2.151,  0.109, -2.257 },
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },
//   };

//   ur3_grip_before_.clear(); ur3_grip_after_.clear();
//   rb3_grip_before_.clear(); rb3_grip_after_.clear();
//   ur3_grip_before_[0] = GripperAction::OPEN;
//   ur3_grip_after_[1]  = GripperAction::CLOSE;
//   ur3_grip_after_[4]  = GripperAction::OPEN;

//   rb3_grip_after_[0]  = GripperAction::OPEN;
//   rb3_grip_before_[2] = GripperAction::PINCH;
//   rb3_grip_after_[2]  = GripperAction::CLOSE;
//   rb3_grip_before_[3] = GripperAction::CLOSE;
//   rb3_grip_after_[8]  = GripperAction::PINCH;
//   rb3_grip_after_[9]  = GripperAction::HOME;

//   sync_points_.clear();
//   sync_points_.push_back(PairSync{/*rb3_idx=*/5, /*ur3_idx=*/3, /*rb3_next=*/6, /*ur3_next=*/4});
//   sync_points_.push_back(PairSync{/*rb3_idx=*/3, /*ur3_idx=*/2, /*rb3_next=*/4, /*ur3_next=*/2});

//   snapshot_base_sequences_hooks_sync();

//   // ---- tactile 파라미터/구독/타이머/서비스 ----
//   tactile_index_topic_     = this->declare_parameter<std::string>("tactile_index_topic", tactile_index_topic_);
//   tactile_middle_topic_    = this->declare_parameter<std::string>("tactile_middle_topic", tactile_middle_topic_);
//   tactile_poll_ms_         = this->declare_parameter<int>("tactile_poll_ms", tactile_poll_ms_);
//   tactile_log_every_ms_    = this->declare_parameter<int>("tactile_log_every_ms", tactile_log_every_ms_);
//   alpha_                   = this->declare_parameter<double>("tactile_ema_alpha", alpha_);
//   baseline_samples_needed_ = this->declare_parameter<int>("tactile_baseline_samples", baseline_samples_needed_);
//   delta_idx_thr_           = this->declare_parameter<int>("tactile_idx_delta_thr", delta_idx_thr_);
//   delta_mid_thr_           = this->declare_parameter<int>("tactile_mid_delta_thr", delta_mid_thr_);
//   {
//     const auto pol = this->declare_parameter<std::string>("tactile_polarity", "increase");
//     polarity_increase_ = (pol == "increase");
//   }

//   {
//     auto qos = rclcpp::SensorDataQoS();
//     sub_idx_tact_ = this->create_subscription<UInt16MA>(
//       tactile_index_topic_, qos,
//       std::bind(&MoveSequenceClient::on_index_tactile_, this, std::placeholders::_1));
//     sub_mid_tact_ = this->create_subscription<UInt16MA>(
//       tactile_middle_topic_, qos,
//       std::bind(&MoveSequenceClient::on_middle_tactile_, this, std::placeholders::_1));
//   }
//   tactile_timer_ = this->create_wall_timer(
//     std::chrono::milliseconds(std::max(10, tactile_poll_ms_)),
//     std::bind(&MoveSequenceClient::tick_tactile_, this)
//   );
//   // 재캘리브레이션 서비스
//   srv_tactile_recalib_ = this->create_service<std_srvs::srv::Trigger>(
//     "tactile_recalib",
//     [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
//            std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
//     {
//       have_baseline_ = false;
//       baseline_cnt_ = 0;
//       baseline_acc_idx_ = baseline_acc_mid_ = 0.0;
//       RCLCPP_WARN(this->get_logger(), "[TACT] Recalibration requested. Collecting %d samples...",
//                   baseline_samples_needed_);
//       resp->success = true;
//       resp->message = "tactile baseline reset; collecting samples";
//     }
//   );

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
//       RCLCPP_INFO(this->get_logger(), "[SYNC] release → fire (RB3=5, UR3=3)");
//       idx_rb3_ = 5; idx_ur3_ = 3;
//       sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;
//       this->fire_both_immediate(idx_rb3_, idx_ur3_);
//       resp->success = true;
//       resp->message = "Proceed fired to (RB3=5, UR3=3).";
//     }
//   );

//   srv_pickblock_ = this->create_service<std_srvs::srv::Trigger>(
//     "zmk_pickblockchkFlag",
//     [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
//           std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
//     {
//       const bool use_A = (idx_rb3_ >= 6 && idx_ur3_ >= 4);
//       block_progress_ = true;
//       auto apply_now = [this, use_A]() { apply_sequence_and_soft_restart(use_A); };

//       if (!rb3_waiting_ && !ur3_waiting_) {
//         apply_now();
//       } else {
//         apply_wait_timer_ = this->create_wall_timer(
//           100ms,
//           [this, apply_now]() {
//             if (!rb3_waiting_ && !ur3_waiting_) {
//               if (apply_wait_timer_) apply_wait_timer_->cancel();
//               apply_now();
//             }
//           }
//         );
//       }

//       resp->success = true;
//       resp->message = use_A
//         ? "Sequence-A will run once then return to MAIN from step 0"
//         : "Sequence-B will run once then return to MAIN from step 0";
//     }
//   );

//   // bringup timer
//   start_timer_ = this->create_wall_timer(300ms, [this] {
//     if (!cli_rb3_->wait_for_service(0s) || !cli_ur3_->wait_for_service(0s) ||
//         !cli_grip_open_->wait_for_service(0s) || !cli_grip_close_->wait_for_service(0s) ||
//         !cli_hand_setangle_->wait_for_service(0s) ||
//         (use_bimanual_srv_ && !client_bimanual_tcppos_->wait_for_service(0s))) {
//       RCLCPP_DEBUG(this->get_logger(), "[WAIT] services ...");
//       return;
//     }
//     const auto right_status = this->resolve_status_topic(right_fjt_base_);
//     const auto left_status  = this->resolve_status_topic(left_fjt_base_);
//     if (right_status.empty() || left_status.empty()) {
//       RCLCPP_ERROR(this->get_logger(),
//         "Could not resolve status topics. right_base=%s left_base=%s",
//         right_fjt_base_.c_str(), left_fjt_base_.c_str());
//       return;
//     }
//     sub_right_status_ = this->create_subscription<GoalStatusArray>(
//       right_status, rclcpp::QoS(50),
//       std::bind(&MoveSequenceClient::on_right_status, this, std::placeholders::_1));
//     sub_left_status_ = this->create_subscription<GoalStatusArray>(
//       left_status, rclcpp::QoS(50),
//       std::bind(&MoveSequenceClient::on_left_status, this, std::placeholders::_1));

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

// // ---- base 스냅샷/복원 ----
// void MoveSequenceClient::snapshot_base_sequences_hooks_sync()
// {
//   base_seq_rb3_ = seq_rb3_;
//   base_seq_ur3_ = seq_ur3_;
//   base_ur3_grip_before_ = ur3_grip_before_;
//   base_ur3_grip_after_  = ur3_grip_after_;
//   base_rb3_grip_before_ = rb3_grip_before_;
//   base_rb3_grip_after_  = rb3_grip_after_;
//   base_sync_points_     = sync_points_;
//   RCLCPP_DEBUG(this->get_logger(), "[BASE] snapshot stored.");
// }

// void MoveSequenceClient::restore_base_sequences_hooks_sync()
// {
//   seq_rb3_ = base_seq_rb3_;
//   seq_ur3_ = base_seq_ur3_;
//   ur3_grip_before_ = base_ur3_grip_before_;
//   ur3_grip_after_  = base_ur3_grip_after_;
//   rb3_grip_before_ = base_rb3_grip_before_;
//   rb3_grip_after_  = base_rb3_grip_after_;
//   sync_points_     = base_sync_points_;
//   RCLCPP_DEBUG(this->get_logger(), "[BASE] restored MAIN sequences/hooks/sync.");
// }

// // ---- restart/cancel ----
// void MoveSequenceClient::start_sequence() {
//   RCLCPP_INFO(this->get_logger(), "[SEQ] start");
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
//   RCLCPP_INFO(this->get_logger(), "[SEQ] reset");
// }
// bool MoveSequenceClient::cancel_all_goals()
// {
//   if (!cancel_right_cli_->wait_for_service(500ms) ||
//       !cancel_left_cli_->wait_for_service(500ms)) {
//     RCLCPP_ERROR(this->get_logger(), "CancelGoal service not available");
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
//     RCLCPP_ERROR(this->get_logger(), "CancelGoal timeout");
//     return false;
//   }
//   try { (void)f_r.get(); (void)f_l.get(); } catch (...) {}
//   std::this_thread::sleep_for(100ms);
//   return true;
// }

// void MoveSequenceClient::apply_sequence_and_soft_restart(bool use_A)
// {
//   if (use_A) load_sequence_A();
//   else       load_sequence_B();

//   auto_restart_once_ = true;
//   exit_when_done_ = false;

//   this->reset_sequence_state();
//   block_progress_ = false;
//   this->start_sequence();
// }

// void MoveSequenceClient::load_sequence_A()
// {
//   seq_rb3_.assign({
//     {-0.04381, -0.45377, 0.13585, 1.75492856,  0.19338248,  1.8102555},
//     {-0.0407,  -0.44568, 0.31162, 1.7228145,   0.17715092,  1.80205245},
//     {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},
//   });
//   seq_ur3_.assign({
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },
//     { -0.140,   -0.410,   0.175,   0.0,   -3.14,  0.0 },
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },
//   });

//   ur3_grip_before_.clear(); ur3_grip_after_.clear();
//   rb3_grip_before_.clear(); rb3_grip_after_.clear();
//   ur3_grip_after_[1]  = GripperAction::OPEN;
//   rb3_grip_after_[0]  = GripperAction::PINCH;
//   rb3_grip_after_[1]  = GripperAction::HOME;
//   sync_points_.clear();

//   RCLCPP_INFO(this->get_logger(), "[SEQ] load A (one-shot)");
// }

// void MoveSequenceClient::load_sequence_B()
// {
//   seq_rb3_.assign({
//     {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245},
//     {-0.04642, -0.44152, 0.121,   1.76208441,  0.24801129,  1.77238186},
//     {-0.0407,  -0.44568, 0.450,   1.7228145,   0.17715092,  1.80205245},
//     {-0.05367, -0.47036, 0.35057, -3.10528981, -0.04380776, 0.015708},
//   });
//   seq_ur3_.assign({
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },
//     { -0.140,   -0.410,   0.175,   0.0,   -3.14,  0.0 },
//     { -0.140,   -0.410,   0.350,   0.0,   -3.14,  0.0 },
//   });

//   ur3_grip_before_.clear(); ur3_grip_after_.clear();
//   rb3_grip_before_.clear(); rb3_grip_after_.clear();
//   ur3_grip_after_[1]  = GripperAction::OPEN;
//   rb3_grip_after_[1]  = GripperAction::PINCH;
//   rb3_grip_after_[2]  = GripperAction::PINCH;
//   rb3_grip_after_[3]  = GripperAction::HOME;
//   sync_points_.clear();

//   RCLCPP_INFO(this->get_logger(), "[SEQ] load B (one-shot)");
// }

// // ---- soft-restart helpers ----
// void MoveSequenceClient::request_soft_restart()
// {
//   restart_pending_.store(true);
//   block_progress_ = true;

//   if (!restart_watchdog_ || restart_watchdog_->is_canceled()) {
//     restart_watchdog_ = this->create_wall_timer(
//       100ms, [this](){
//         if (!restart_pending_.load()) { restart_watchdog_->cancel(); return; }
//         if (!rb3_waiting_ && !ur3_waiting_) {
//           RCLCPP_INFO(this->get_logger(), "[SEQ] soft-restart now");
//           restart_watchdog_->cancel();
//           do_restart_now();
//         }
//       }
//     );
//   }
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
//   RCLCPP_INFO(this->get_logger(), "[SEQ] soft-restart → step0");
//   this->reset_sequence_state();
//   restart_pending_.store(false);
//   block_progress_ = false;
//   this->start_sequence();
// }

// // ---- first pair ----
// void MoveSequenceClient::prepare_and_fire_first_pair() {
//   if (restart_pending_.load()) return;

//   const bool need_rb3 = rb3_grip_before_.count(0) && rb3_grip_before_[0] != GripperAction::NONE;
//   const bool need_ur3 = ur3_grip_before_.count(0) && ur3_grip_before_[0] != GripperAction::NONE;

//   if (!need_rb3 && !need_ur3) {
//     sync_inflight_ = std::make_pair(0u, 0u);
//     rb3_inflight_done_ = ur3_inflight_done_ = false;
//     this->fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
//     return;
//   }

//   auto pending = std::make_shared<std::atomic<int>>(0);
//   if (need_rb3) pending->fetch_add(1);
//   if (need_ur3) pending->fetch_add(1);

//   auto after = [this, pending]() {
//     if (pending->fetch_sub(1) == 1) {
//       sync_inflight_ = std::make_pair(0u, 0u);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;
//       this->fire_both_now(/*rb3_idx=*/0, /*ur3_idx=*/0, /*apply_before_hooks=*/false);
//     }
//   };

//   if (need_rb3) this->request_rb_hand(rb3_grip_before_[0], after);
//   if (need_ur3) this->request_ur_gripper(ur3_grip_before_[0], after);
// }

// // ---- concurrent fire ----
// void MoveSequenceClient::fire_both_now(size_t rb3_idx, size_t ur3_idx, bool apply_before_hooks)
// {
//   if (restart_pending_.load()) return;

//   if (!apply_before_hooks) {
//     sync_fire_timer_ = this->create_wall_timer(
//       0ms,
//       [this, rb3_idx, ur3_idx](){
//         if (restart_pending_.load()) { sync_fire_timer_->cancel(); return; }

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
//   if (do_rb3_before) this->request_rb_hand(rb3_grip_before_[rb3_idx], [after_one_done](){});
//   if (do_ur3_before) this->request_ur_gripper(ur3_grip_before_[ur3_idx], [after_one_done](){});
// }

// void MoveSequenceClient::fire_both_immediate(size_t rb3_idx, size_t ur3_idx)
// {
//   if (restart_pending_.load()) return;

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
//   if (!resp || !resp->success) {
//     RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] move_tcppos failed");
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

//   client_bimanual_tcppos_->async_send_request(
//     req,
//     [this](rclcpp::Client<dual_arm_msg::srv::Movetcppos>::SharedFuture f){
//       bool ok = false;
//       try { auto resp = f.get(); ok = resp && resp->success; } catch (...) {}
//       if (!ok) RCLCPP_ERROR(this->get_logger(), "[BIMANUAL] request failed");
//     });
// }

// // ---- RB3 ----
// void MoveSequenceClient::start_rb3_step(size_t i)
// {
//   if (i >= seq_rb3_.size()) { if (!rb3_done_all_) { rb3_done_all_ = true; this->maybe_finish(); } return; }
//   if (rb3_grip_before_.count(i) && rb3_grip_before_[i] != GripperAction::NONE) {
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

//   RCLCPP_INFO(this->get_logger(), "[STEP] RB3  idx=%zu", i);
//   cli_rb3_->async_send_request(
//     req,
//     [this, i](rclcpp::Client<RB3Srv>::SharedFuture future) {
//       auto resp = future.get();
//       if (!resp || !resp->success) {
//         RCLCPP_ERROR(this->get_logger(), "[RB3] send fail at idx=%zu", i);
//       }
//     });
// }
// void MoveSequenceClient::advance_rb3_after_done(size_t just_finished_idx)
// {
//   if (restart_pending_.load() || block_progress_) {
//     rb3_waiting_ = false;
//     maybe_restart_after_idle();
//     return;
//   }
//   if (rb3_grip_after_.count(just_finished_idx) &&
//       rb3_grip_after_[just_finished_idx] != GripperAction::NONE) {
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

//   RCLCPP_INFO(this->get_logger(), "[STEP] UR3e idx=%zu", i);
//   cli_ur3_->async_send_request(
//     req,
//     [this, i](rclcpp::Client<UR3Srv>::SharedFuture future) {
//       auto resp = future.get();
//       if (!resp || !resp->success) {
//         RCLCPP_ERROR(this->get_logger(), "[UR3e] send fail at idx=%zu", i);
//       }
//     });
// }
// void MoveSequenceClient::advance_ur3_after_done(size_t just_finished_idx)
// {
//   if (restart_pending_.load() || block_progress_) {
//     ur3_waiting_ = false;
//     maybe_restart_after_idle();
//     return;
//   }
//   if (ur3_grip_after_.count(just_finished_idx) &&
//       ur3_grip_after_[just_finished_idx] != GripperAction::NONE) {
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
//       RCLCPP_INFO(this->get_logger(), "[SYNC] RB3 idx=%zu", sp.rb3_idx);
//       this->run_after_hook_barrier_only(/*is_left=*/false, just_finished_idx);
//     }
//     if ( is_left && just_finished_idx == sp.ur3_idx) {
//       sp.reached_ur3 = true; matched_any = true;
//       RCLCPP_INFO(this->get_logger(), "[SYNC] UR3e idx=%zu", sp.ur3_idx);
//       this->run_after_hook_barrier_only(/*is_left=*/true, just_finished_idx);
//     }

//     if (sp.reached_rb3 && sp.reached_ur3 && !sp.fired) {
//       sp.fired = true;
//       if (restart_pending_.load() || block_progress_) return true;

//       idx_rb3_ = sp.rb3_next;
//       idx_ur3_ = sp.ur3_next;

//       sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;

//       RCLCPP_INFO(this->get_logger(), "[SYNC] FIRE → RB3=%zu UR3e=%zu", idx_rb3_, idx_ur3_);
//       this->fire_both_now(idx_rb3_, idx_ur3_, /*apply_before_hooks=*/false);
//       return true;
//     }
//   }
//   return matched_any;
// }

// bool MoveSequenceClient::try_fire_next_sync_after_inflight_pair(size_t just_rb3_idx, size_t just_ur3_idx)
// {
//   for (auto &sp : sync_points_) {
//     if (sp.rb3_idx == just_rb3_idx && sp.ur3_idx == just_ur3_idx) {
//       if (sp.fired) return false;
//       sp.fired = true;

//       if (restart_pending_.load() || block_progress_) return true;

//       idx_rb3_ = sp.rb3_next;
//       idx_ur3_ = sp.ur3_next;

//       sync_inflight_ = std::make_pair(idx_rb3_, idx_ur3_);
//       rb3_inflight_done_ = ur3_inflight_done_ = false;

//       RCLCPP_INFO(this->get_logger(), "[SYNC] POST → RB3=%zu UR3e=%zu", idx_rb3_, idx_ur3_);
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
//   (void)rb3_idx; (void)ur3_idx;

//   if (rb3_idx == 4 && ur3_idx == 2) {
//     hold_until_degcheck_ = true;
//     sync_inflight_.reset();
//     RCLCPP_INFO(this->get_logger(), "[SYNC] HOLD (RB3=4, UR3=2) call /zmk_DegcheckFlag");
//     return;
//   }

//   if (try_fire_next_sync_after_inflight_pair(rb3_idx, ur3_idx)) {
//     return;
//   }

//   sync_inflight_.reset();
//   rb3_inflight_done_ = ur3_inflight_done_ = false;

//   if (restart_pending_.load() || block_progress_) {
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
//     if (auto_restart_once_) {
//       RCLCPP_INFO(this->get_logger(), "[SEQ] oneshot done → restore main");
//       auto_restart_once_ = false;
//       restore_base_sequences_hooks_sync();
//       this->reset_sequence_state();
//       this->start_sequence();
//       return;
//     }
//     RCLCPP_INFO(this->get_logger(), "[SEQ] done");
//     if (exit_when_done_ && !shutdown_scheduled_) {
//       shutdown_scheduled_ = true;
//       shutdown_timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(200),
//         [this]() {
//           RCLCPP_INFO(this->get_logger(), "[SEQ] shutdown");
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
//     RCLCPP_WARN(this->get_logger(), "[UR3e] gripper svc not available");
//     if (on_done) on_done();
//     return;
//   }
//   cli->async_send_request(
//     req,
//     [this, on_done](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
//       (void)future;
//       if (on_done) on_done();
//     });
// }

// void MoveSequenceClient::request_rb_hand(GripperAction action, std::function<void()> on_done)
// {
//   if (action == GripperAction::NONE) { if (on_done) on_done(); return; }
//   if (!cli_hand_setangle_->wait_for_service(0s)) {
//     RCLCPP_WARN(this->get_logger(), "[RB3] hand svc not available");
//     if (on_done) on_done();
//     return;
//   }

//   auto req = std::make_shared<HandSetAngleSrv::Request>();
//   if (action == GripperAction::CLOSE)
//   { req->angle0 = 0; req->angle1 = 0; req->angle2 = 645; req->angle3 = 671;  req->angle4 = 260;  req->angle5 = 0; }
//   else if(action == GripperAction::PINCH)
//   { req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000; req->angle3 = 1000; req->angle4 = 400;  req->angle5 = 0; }
//   else if(action == GripperAction::HOME)
//   { req->angle0 = 0; req->angle1 = 0; req->angle2 = 0; req->angle3 = 0; req->angle4 = 0;  req->angle5 = 1000; }
//   else // OPEN
//   { req->angle0 = 1000; req->angle1 = 1000; req->angle2 = 1000; req->angle3 = 1000; req->angle4 = 0; req->angle5 = 1000; }

//   req->hand_id = 1; req->status  = "set_angle";

//   cli_hand_setangle_->async_send_request(
//     req,
//     [this, on_done](rclcpp::Client<HandSetAngleSrv>::SharedFuture future) {
//       (void)future;
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
//     }
//     if (rb3_seen_active_ && !active) {
//       rb3_waiting_ = false;
//       RCLCPP_INFO(this->get_logger(), "[DONE] RB3  idx=%zu", idx_rb3_);

//       if (sync_inflight_.has_value() && idx_rb3_ == sync_inflight_->first) {
//         this->run_after_hook_barrier_only(/*is_left=*/false, idx_rb3_);
//         rb3_inflight_done_ = true;
//         this->check_inflight_and_advance_after_both_done();
//         return;
//       }
//       if (this->handle_sync_reached(/*is_left=*/false, idx_rb3_)) return;

//       this->advance_rb3_after_done(idx_rb3_);
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
//     }
//     if (ur3_seen_active_ && !active) {
//       ur3_waiting_ = false;
//       RCLCPP_INFO(this->get_logger(), "[DONE] UR3e idx=%zu", idx_ur3_);

//       if (sync_inflight_.has_value() && idx_ur3_ == sync_inflight_->second) {
//         this->run_after_hook_barrier_only(/*is_left=*/true, idx_ur3_);
//         ur3_inflight_done_ = true;
//         this->check_inflight_and_advance_after_both_done();
//         return;
//       }
//       if (this->handle_sync_reached(/*is_left=*/true, idx_ur3_)) return;

//       this->advance_ur3_after_done(idx_ur3_);
//       maybe_restart_after_idle();
//     }
//   }
// }

// // ---- tactile (EMA + Baseline + Polarity + Delta Thr) ----
// void MoveSequenceClient::on_index_tactile_(const UInt16MA::SharedPtr msg)
// {
//   int s = 0; for (auto v : msg->data) s += static_cast<int>(v);
//   last_idx_sum_.store(s, std::memory_order_relaxed);

//   // EMA 업데이트
//   if (baseline_cnt_ == 0 && !have_baseline_) { ema_idx_ = static_cast<double>(s); }
//   else { ema_idx_ = alpha_ * static_cast<double>(s) + (1.0 - alpha_) * ema_idx_; }

//   // baseline 수집 중이면 누적
//   if (!have_baseline_) {
//     baseline_acc_idx_ += static_cast<double>(s);
//   }
// }
// void MoveSequenceClient::on_middle_tactile_(const UInt16MA::SharedPtr msg)
// {
//   int s = 0; for (auto v : msg->data) s += static_cast<int>(v);
//   last_mid_sum_.store(s, std::memory_order_relaxed);

//   // EMA 업데이트
//   if (baseline_cnt_ == 0 && !have_baseline_) { ema_mid_ = static_cast<double>(s); }
//   else { ema_mid_ = alpha_ * static_cast<double>(s) + (1.0 - alpha_) * ema_mid_; }

//   // baseline 수집 중이면 누적
//   if (!have_baseline_) {
//     baseline_acc_mid_ += static_cast<double>(s);
//   }
// }
// void MoveSequenceClient::tick_tactile_()
// {
//   // baseline 없으면 샘플 모으기/완료
//   if (!have_baseline_) {
//     ++baseline_cnt_;
//     if (baseline_cnt_ >= std::max(5, baseline_samples_needed_)) {
//       baseline_idx_ = baseline_acc_idx_ / static_cast<double>(baseline_cnt_);
//       baseline_mid_ = baseline_acc_mid_ / static_cast<double>(baseline_cnt_);
//       have_baseline_ = true;
//       RCLCPP_WARN(this->get_logger(),
//         "[TACT] Baseline locked: idx=%.1f mid=%.1f (N=%d, alpha=%.2f, pol=%s, thrI=%d thrM=%d)",
//         baseline_idx_, baseline_mid_, baseline_cnt_, alpha_,
//         polarity_increase_ ? "increase" : "decrease", delta_idx_thr_, delta_mid_thr_);
//     } else {
//       // 진행 상황 로그
//       auto now0 = std::chrono::steady_clock::now();
//       if (last_tactile_log_tp_.time_since_epoch().count() == 0 ||
//           std::chrono::duration_cast<std::chrono::milliseconds>(now0 - last_tactile_log_tp_).count() >= tactile_log_every_ms_) {
//         last_tactile_log_tp_ = now0;
//         RCLCPP_INFO(this->get_logger(),
//           "[TACT] calibrating... idx_sum=%d mid_sum=%d cnt=%d/%d",
//           last_idx_sum_.load(), last_mid_sum_.load(), baseline_cnt_, baseline_samples_needed_);
//       }
//       return;
//     }
//   }

//   // baseline 보유 → 판정
//   const double di = ema_idx_ - baseline_idx_;
//   const double dm = ema_mid_ - baseline_mid_;
//   bool present_now;

//   if (polarity_increase_) {
//     present_now = (di > static_cast<double>(delta_idx_thr_)) || (dm > static_cast<double>(delta_mid_thr_));
//   } else {
//     present_now = ((-di) > static_cast<double>(delta_idx_thr_)) || ((-dm) > static_cast<double>(delta_mid_thr_));
//   }

//   if (present_now != last_obj_present_) {
//     last_obj_present_ = present_now;
//     RCLCPP_INFO(this->get_logger(),
//       "[TACT] OBJ=%s idx(ema=%.1f base=%.1f d=%.1f thr=%d) mid(ema=%.1f base=%.1f d=%.1f thr=%d) (RB3=%zu UR3=%zu)",
//       present_now ? "YES" : "NO",
//       ema_idx_, baseline_idx_, di, delta_idx_thr_,
//       ema_mid_, baseline_mid_, dm, delta_mid_thr_,
//       idx_rb3_, idx_ur3_);
//   }

//   // 주기 로그
//   auto now = std::chrono::steady_clock::now();
//   if (last_tactile_log_tp_.time_since_epoch().count() == 0 ||
//       std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tactile_log_tp_).count() >= tactile_log_every_ms_) {
//     last_tactile_log_tp_ = now;
//     RCLCPP_INFO(this->get_logger(),
//       "[TACT] OBJ=%s idx(ema=%.1f base=%.1f d=%.1f thr=%d) mid(ema=%.1f base=%.1f d=%.1f thr=%d) (RB3=%zu UR3=%zu)",
//       present_now ? "YES" : "NO",
//       ema_idx_, baseline_idx_, di, delta_idx_thr_,
//       ema_mid_, baseline_mid_, dm, delta_mid_thr_,
//       idx_rb3_, idx_ur3_);
//   }
// }

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MoveSequenceClient>());
//   rclcpp::shutdown();
//   return 0;
// }
