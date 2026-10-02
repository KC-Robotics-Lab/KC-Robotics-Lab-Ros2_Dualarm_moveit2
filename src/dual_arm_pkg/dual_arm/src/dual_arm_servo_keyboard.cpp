#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "control_msgs/msg/joint_jog.hpp"

// --- POSIX non-blocking keyboard ---
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <array>

using namespace std::chrono_literals;

namespace kb {

// Simple RAII to set terminal to raw mode
class TerminalRaw {
public:
  TerminalRaw() : active_(false) {
    if (tcgetattr(STDIN_FILENO, &orig_) == 0) {
      termios raw = orig_;
      raw.c_lflag &= ~(ICANON | ECHO);
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;
      if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
        // set non-blocking
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        active_ = true;
      }
    }
  }
  ~TerminalRaw() {
    if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
  }
private:
  termios orig_{};
  bool active_;
};

inline int getch() {
  unsigned char c;
  ssize_t n = ::read(STDIN_FILENO, &c, 1);
  if (n == 1) return static_cast<int>(c);
  return -1;
}

} // namespace kb

class DualArmServoKeyboard : public rclcpp::Node {
public:
  DualArmServoKeyboard() : Node("dual_arm_servo_keyboard"),
                           raw_term_(), stop_requested_(false) {
    // ---- Parameters ----
    ur_ns_ = declare_parameter<std::string>("ur_servo_ns", "ur/servo_node");
    rb_ns_ = declare_parameter<std::string>("rb_servo_ns", "rb/servo_node");

    ur_command_frame_ = declare_parameter<std::string>("ur_command_frame", "base_ur");
    rb_command_frame_ = declare_parameter<std::string>("rb_command_frame", "base_rb");

    rate_hz_ = declare_parameter<double>("rate_hz", 50.0); // 50Hz
    ur_lin_step_ = declare_parameter<double>("ur_linear_step", 0.05);   // m/s per keypress (held)
    ur_ang_step_ = declare_parameter<double>("ur_angular_step", 0.15);  // rad/s per keypress (held)

    // RB joint list (order matters for key mapping below)
    rb_joint_names_ = declare_parameter<std::vector<std::string>>(
      "rb_joint_names",
      std::vector<std::string>({"joint1_rb","joint2_rb","joint3_rb","joint4_rb","joint5_rb","joint6_rb"}));
    rb_joint_step_ = declare_parameter<double>("rb_joint_step", 0.2);   // rad/s per keypress (held)
    rb_zero_on_idle_ = declare_parameter<bool>("rb_zero_on_idle", true); // true면 눌리지 않으면 0으로 보냄

    // Deadman & safety
    show_help_on_start_ = declare_parameter<bool>("show_help_on_start", true);

    // ---- Publishers ----
    pub_ur_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/" + ur_ns_ + "/delta_twist_cmds", 10);
    pub_rb_jjog_ = create_publisher<control_msgs::msg::JointJog>(
        "/" + rb_ns_ + "/delta_joint_cmds", 10);

    // ---- Timer ----
    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DualArmServoKeyboard::onTimer, this));

    // ---- Help ----
    if (show_help_on_start_) printHelp();

    RCLCPP_INFO(get_logger(), "Keyboard teleop started (%.1f Hz). Press 'h' to show help, 'Space' to zero, 'Ctrl+C' to exit.", rate_hz_);
  }

private:
  // Key map:
  // UR (Cartesian):
  //   Linear:  w/s: +X/-X,  a/d: +Y/-Y,  r/f: +Z/-Z
  //   Angular: q/e: +Yaw/-Yaw (Z), z/x: +Pitch/-Pitch (Y), c/v: +Roll/-Roll (X)
  //
  // RB (Joint velocities; 6 joints):
  //   t/g: j1 +/-
  //   y/h: j2 +/-
  //   u/j: j3 +/-
  //   i/k: j4 +/-
  //   o/l: j5 +/-
  //   p/;: j6 +/-
  //
  // Common:
  //   Space: all zero (halt)
  //   h: help
  //   ESC: zero (one-shot)
  //   Ctrl+C: exit (handled by terminal)

  void printHelp() {
    std::cout <<
      "\n=== Dual-Arm Servo Keyboard ===\n"
      "[UR Cartesian]\n"
      "  w/s: +X / -X    a/d: +Y / -Y    r/f: +Z / -Z\n"
      "  q/e: +Yaw/-Yaw  z/x: +Pitch/-Pitch  c/v: +Roll/-Roll\n"
      "  steps: linear=" << ur_lin_step_ << " m/s, angular=" << ur_ang_step_ << " rad/s\n"
      "[RB Joint velocities]\n"
      "  t/g: j1 +/-   y/h: j2 +/-   u/j: j3 +/-\n"
      "  i/k: j4 +/-   o/l: j5 +/-   p/;: j6 +/-\n"
      "  step: " << rb_joint_step_ << " rad/s\n"
      "[Common]\n"
      "  Space: ZERO ALL   h: help   ESC: zero once   Ctrl+C: exit\n"
      "--------------------------------------------\n";
  }

  void onTimer() {
    if (stop_requested_) {
      sendZeroAll();
      rclcpp::shutdown();
      return;
    }

    // 1) Read all pending keys
    processKeys();

    // 2) Publish UR Twist
    publishUR();

    // 3) Publish RB JointJog
    publishRB();
  }

  void processKeys() {
    // Reset commands each cycle (hold-to-run behavior).
    // If you prefer "toggle" behavior, remove these zeroing lines and adjust accumulation below.
    ur_twist_cmd_ = {0,0,0, 0,0,0};
    rb_joint_cmd_.assign(rb_joint_names_.size(), 0.0);

    int c = 0;
    while ((c = kb::getch()) != -1) {
      switch (c) {
        // UR linear
        case 'w': ur_twist_cmd_[0] += ur_lin_step_; break; // +X
        case 's': ur_twist_cmd_[0] -= ur_lin_step_; break; // -X
        case 'a': ur_twist_cmd_[1] += ur_lin_step_; break; // +Y
        case 'd': ur_twist_cmd_[1] -= ur_lin_step_; break; // -Y
        case 'r': ur_twist_cmd_[2] += ur_lin_step_; break; // +Z
        case 'f': ur_twist_cmd_[2] -= ur_lin_step_; break; // -Z
        // UR angular
        case 'q': ur_twist_cmd_[5] += ur_ang_step_; break; // +Yaw (Z)
        case 'e': ur_twist_cmd_[5] -= ur_ang_step_; break; // -Yaw (Z)
        case 'z': ur_twist_cmd_[4] += ur_ang_step_; break; // +Pitch (Y)
        case 'x': ur_twist_cmd_[4] -= ur_ang_step_; break; // -Pitch (Y)
        case 'c': ur_twist_cmd_[3] += ur_ang_step_; break; // +Roll (X)
        case 'v': ur_twist_cmd_[3] -= ur_ang_step_; break; // -Roll (X)

        // RB joints: t/g, y/h, u/j, i/k, o/l, p/;
        case 't': setJointVel(0, +rb_joint_step_); break;
        case 'g': setJointVel(0, -rb_joint_step_); break;

        case 'y': setJointVel(1, +rb_joint_step_); break;
        case 'h': setJointVel(1, -rb_joint_step_); break;

        case 'u': setJointVel(2, +rb_joint_step_); break;
        case 'j': setJointVel(2, -rb_joint_step_); break;

        case 'i': setJointVel(3, +rb_joint_step_); break;
        case 'k': setJointVel(3, -rb_joint_step_); break;

        case 'o': setJointVel(4, +rb_joint_step_); break;
        case 'l': setJointVel(4, -rb_joint_step_); break;

        case 'p': setJointVel(5, +rb_joint_step_); break;
        case ';': setJointVel(5, -rb_joint_step_); break;

        // common
        case ' ': sendZeroAll(); break;       // Space: zero
        case 27:  sendZeroAll(); break;       // ESC: zero
        case '?': printHelp(); break;         // 도움말 키를 '?'로 변경
        case 'H': printHelp(); break;         // (선택) Shift+H도 도움말

        default: break; // ignore others
      }
    }

    // If rb_zero_on_idle_ and no joint keys pressed, keep zeros (already done by reset).
    // If you want "last command hold", remove the rb_joint_cmd_ reset at top.
  }

  void setJointVel(size_t idx, double v) {
    if (idx < rb_joint_cmd_.size()) rb_joint_cmd_[idx] = v;
  }

  void publishUR() {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = ur_command_frame_;
    msg.twist.linear.x  = ur_twist_cmd_[0];
    msg.twist.linear.y  = ur_twist_cmd_[1];
    msg.twist.linear.z  = ur_twist_cmd_[2];
    msg.twist.angular.x = ur_twist_cmd_[3];
    msg.twist.angular.y = ur_twist_cmd_[4];
    msg.twist.angular.z = ur_twist_cmd_[5];
    pub_ur_twist_->publish(msg);
  }

  void publishRB() {
    control_msgs::msg::JointJog jj;
    jj.header.stamp = now();
    jj.header.frame_id = rb_command_frame_;
    jj.joint_names = rb_joint_names_;
    jj.velocities = rb_joint_cmd_;
    pub_rb_jjog_->publish(jj);
  }

  void sendZeroAll() {
    ur_twist_cmd_ = {0,0,0, 0,0,0};
    rb_joint_cmd_.assign(rb_joint_names_.size(), 0.0);
    publishUR();
    publishRB();
  }

  rclcpp::Time now() { return this->get_clock()->now(); }

private:
  // Params
  std::string ur_ns_, rb_ns_;
  std::string ur_command_frame_, rb_command_frame_;
  double rate_hz_;
  double ur_lin_step_, ur_ang_step_;
  std::vector<std::string> rb_joint_names_;
  double rb_joint_step_;
  bool rb_zero_on_idle_;
  bool show_help_on_start_;

  // Command state (hold-to-run each cycle)
  // [linx, liny, linz, roll, pitch, yaw]
  std::array<double,6> ur_twist_cmd_ {0,0,0, 0,0,0};
  std::vector<double> rb_joint_cmd_;

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_ur_twist_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr pub_rb_jjog_;
  rclcpp::TimerBase::SharedPtr timer_;

  // terminal raw mode
  kb::TerminalRaw raw_term_;
  bool stop_requested_;
};

int main(int argc, char** argv) {
  // Safer Ctrl+C handling (let rclcpp handle SIGINT)
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DualArmServoKeyboard>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
