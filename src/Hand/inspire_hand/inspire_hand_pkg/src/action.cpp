#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <functional>

#include "serial/serial.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "inspire_hand_interface/msg/angleact.hpp"
#include "inspire_hand_interface/action/set_angle.hpp"

using SetAngle = inspire_hand_interface::action::SetAngle;
using GoalHandleSetAngle = rclcpp_action::ServerGoalHandle<SetAngle>;

#define angleset	        0x05CE

#define getangle_act        0x060A

std::mutex serial_mutex;
serial::Serial ros_ser;

unsigned char send_buffer[64] = {0};
unsigned char recv_buffer[256] = {0};

int get_angles[6] = {0};

class HandControlActionServer : public rclcpp::Node
{
public:
  HandControlActionServer()
  : Node("hand_control_action_server")
    {
        // using namespace std::placeholders;

        // try
        // {
        //     ros_ser.setPort("/dev/ttyUSB0");
        //     ros_ser.setBaudrate(115200);
        //     serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        //     ros_ser.setTimeout(to);
        //     ros_ser.open();
        //     if (ros_ser.isOpen()) {
        //         RCLCPP_INFO(this->get_logger(), "Serial port opened.");
        //     } else {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
        //         rclcpp::shutdown();
        //         return;
        //     }
        //     // ros_ser.flush();
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // }
        // catch (serial::IOException &e)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Serial exception: %s", e.what());
        //     rclcpp::shutdown();
        //     return;
        // }

        // get_angleact = this->create_publisher<inspire_hand_interface::msg::Angleact>("hand_angleact", 10);

        // // angleact subscriber
        // sub_angleact_ = this->create_subscription<inspire_hand_interface::msg::Angleact>(
        // "hand_angleact", 10,
        // std::bind(&HandControlActionServer::angleact_callback, this, _1));

        // // 액션 서버
        // action_server_ = rclcpp_action::create_server<SetAngle>(
        // this,
        // "set_angle",
        // std::bind(&HandControlActionServer::handle_goal, this, _1, _2),
        // std::bind(&HandControlActionServer::handle_cancel, this, _1),
        // std::bind(&HandControlActionServer::handle_accepted, this, _1));

        // timer1_ = this->create_wall_timer(
        //     std::chrono::milliseconds(1),
        //     std::bind(&HandControlActionServer::publish_message, this)
        // );
    }

private:
    // 액션 서버
    // rclcpp_action::Server<SetAngle>::SharedPtr action_server_;

    // // pub/sub
    // rclcpp::Subscription<inspire_hand_interface::msg::Angleact>::SharedPtr sub_angleact_;
    // rclcpp::Publisher<inspire_hand_interface::msg::Angleact>::SharedPtr get_angleact;
    // rclcpp::TimerBase::SharedPtr timer1_;

    // uint16_t current_angles_[6] = {0};

    // // goal 수락
    // rclcpp_action::GoalResponse handle_goal(
    //     const rclcpp_action::GoalUUID &,
    //     std::shared_ptr<const SetAngle::Goal> goal)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Goal received");
    //     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    // }

    // // 취소
    // rclcpp_action::CancelResponse handle_cancel(
    //     const std::shared_ptr<GoalHandleSetAngle>)
    // {
    //     RCLCPP_WARN(this->get_logger(), "Goal canceled");
    //     return rclcpp_action::CancelResponse::ACCEPT;
    // }

    // // 승인 후 실행
    // void handle_accepted(const std::shared_ptr<GoalHandleSetAngle> goal_handle)
    // {
    //     std::thread{std::bind(&HandControlActionServer::execute, this, goal_handle)}.detach();
    // }

    // // 액션 실행
    // void execute(const std::shared_ptr<GoalHandleSetAngle> goal_handle)
    // {
    //     const auto goal = goal_handle->get_goal();
    //     auto feedback = std::make_shared<SetAngle::Feedback>();
    //     auto result = std::make_shared<SetAngle::Result>();

    //     // 실제 모터 명령 보내는 send_set_command() 는 너가 넣으면 됨
    //     std::array<uint16_t, 6> target = {
    //     goal->angle0, goal->angle1, goal->angle2,
    //     goal->angle3, goal->angle4, goal->angle5
    //     };

    //     bool success = send_set_command(goal->hand_id, target, angleset, "set_angle");
    //     if (!success)
    //     {
    //       RCLCPP_ERROR(this->get_logger(), "Failed to send set_angle command");
    //       result->success = false;
    //       goal_handle->abort(result);
    //       return;
    //     }

    //     RCLCPP_INFO(this->get_logger(), "set_angle command sent, monitoring...");

    //     rclcpp::Rate rate(10);
    //     int loop_count = 0;
    //     const int max_loop = 100; // 10초

    //     while (rclcpp::ok() && loop_count++ < max_loop)
    //     {
    //     // 피드백
    //     for(int i=0; i<6; i++)
    //         feedback->current_angles[i] = current_angles_[i];
    //     goal_handle->publish_feedback(feedback);

    //     // 목표 도달 확인
    //     bool reached = true;
    //     for(int i=0; i<6; i++)
    //     {
    //         if (std::abs(static_cast<int>(current_angles_[i]) - static_cast<int>(target[i])) > 15) {
    //             reached = false;
    //             break;
    //         }
    //     }
    //     if (reached) {
    //         RCLCPP_INFO(this->get_logger(), "Goal reached!");
    //         result->success = true;
    //         goal_handle->succeed(result);
    //         return;
    //     }
    //     rate.sleep();
    //     }

    //     // timeout
    //     result->success = false;
    //     goal_handle->abort(result);
    //     RCLCPP_WARN(this->get_logger(), "Goal timeout / aborted");
    // }

    // // sub 콜백
    // void angleact_callback(const inspire_hand_interface::msg::Angleact::SharedPtr msg)
    // {
    //     for (int i=0; i<6; i++)
    //     {
    //     current_angles_[i] = msg->angle[i];
    //     // RCLCPP_INFO(this->get_logger(), "msg->angle[%d] : %d", i, current_angles_[i]);
    //     }
    // }

    // void publish_message()
    // {
    //     auto angleact_msg = inspire_hand_interface::msg::Angleact();
    //     auto angle_ = read_registers(1, getangle_act, 12);
    //     for (int i = 0; i < 6; i++)
    //     {
    //         angleact_msg.angle[i] = angle_[i];
    //         get_angles[i] = angle_[i];
    //     }
    //     get_angleact->publish(angleact_msg);
    // }

    // std::vector<uint16_t> parse_packets(const std::vector<unsigned char>& buffer, uint8_t len)
    // {
    //     std::vector<uint16_t> data;

    //     for (size_t offset = 0; offset + 8 <= buffer.size(); ++offset)
    //     {
    //         if (buffer[offset] == 0x90 && buffer[offset + 1] == 0xEB && buffer[offset + 4] == 0x11)
    //         {
    //             size_t total_len = 7 + len + 1;

    //             if (offset + total_len > buffer.size())
    //             {
    //                 // 패킷이 부족함
    //                 continue;
    //             }

    //             for (size_t i = 0; i < 6; ++i)
    //             {
    //                 size_t low_index = offset + 7 + i * 2;
    //                 size_t high_index = offset + 8 + i * 2;
    //                 uint16_t value = buffer[low_index] | (buffer[high_index] << 8);
    //                 data.push_back(value);
    //             }

    //             return data;  // 유효한 패킷 파싱 완료
    //         }
    //     }

    //     return {};  // 유효한 패킷 없음
    // }

    // std::vector<uint16_t> read_registers(uint8_t hand_id, uint16_t reg, uint8_t len)
    // {
    //     std::lock_guard<std::mutex> lock(serial_mutex);
    //     std::memset(recv_buffer, 0, sizeof(recv_buffer));

    //     if (!ros_ser.isOpen()) {
    //         RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
    //         return {};
    //       }
    //     //   else RCLCPP_INFO(this->get_logger(), "Serial port is open");

    //     try
    //     {
    //         uint8_t checksum = 0;
    //         send_buffer[0] = 0xEB;
    //         send_buffer[1] = 0x90;
    //         send_buffer[2] = hand_id;
    //         send_buffer[3] = 0x04;
    //         send_buffer[4] = 0x11;
    //         send_buffer[5] = reg & 0xFF;
    //         send_buffer[6] = reg >> 8;
    //         send_buffer[7] = len;
    //         for (int i = 2; i < 8; i++) checksum += send_buffer[i];
    //         send_buffer[8] = checksum;

    //         ros_ser.write(send_buffer, 9);
    //         ros_ser.flush();

    //         size_t expected_size = 7 + len + 1;
    //         std::vector<unsigned char> recv_buffer;
    //         recv_buffer.reserve(expected_size);  // 여유를 둠
    //         // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "expected_size : %d", expected_size);

    //         auto start = std::chrono::steady_clock::now();
    //         while (std::chrono::duration_cast<std::chrono::milliseconds>(
    //                 std::chrono::steady_clock::now() - start).count() < 50)
    //         {
    //             if (ros_ser.available())
    //             {
    //                 // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "available : %d", ros_ser.available());
    //                 // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "2222222");
    //                 size_t to_read = ros_ser.available();
    //                 std::vector<unsigned char> temp_buf(to_read);
    //                 size_t read_len = ros_ser.read(temp_buf.data(), to_read);
    //                 // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "read_len : %d", read_len);
    //                 recv_buffer.insert(recv_buffer.end(), temp_buf.begin(), temp_buf.begin() + read_len);

    //                 auto parsed = parse_packets(recv_buffer, len);
    //                 if (!parsed.empty())
    //                 {
    //                     // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "333333");
    //                     return parsed;
    //                 }
    //             }
    //         }

    //         RCLCPP_WARN(this->get_logger(), "Timeout or invalid packet during read_registers");
    //         return {};
    //     }
    //     catch (const std::exception &e) {
    //         RCLCPP_ERROR(rclcpp::get_logger("HandInterface"), "Serial exception in read_registers: %s", e.what());
    //         return {};
    //     }
    // }

    // bool send_set_command(uint8_t hand_id, const std::array<uint16_t, 6>& values, uint16_t reg, const std::string& context)
    // {
    //     std::lock_guard<std::mutex> lock(serial_mutex);

    //     if (!ros_ser.isOpen()) {
    //         RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "Serial port not open. Skipping send_set_one_command.");
    //         return false;
    //     }

    //     try
    //     {
    //         // 버퍼 비우기
    //         while (ros_ser.available())
    //         {
    //             std::vector<unsigned char> flush_buf(ros_ser.available());
    //             ros_ser.read(flush_buf.data(), flush_buf.size());
    //         }

    //         // 패킷 작성
    //         uint8_t checksum = 0;
    //         send_buffer[0] = 0xEB;
    //         send_buffer[1] = 0x90;
    //         send_buffer[2] = hand_id;
    //         send_buffer[3] = 0x0F;
    //         send_buffer[4] = 0x12;
    //         send_buffer[5] = reg & 0xFF;
    //         send_buffer[6] = reg >> 8;
    //         for (size_t i = 0; i < 6; ++i)
    //         {
    //             send_buffer[7 + i * 2] = values[i] & 0xFF;
    //             send_buffer[8 + i * 2] = (values[i] >> 8) & 0xFF;
    //         }
    //         for (int i = 2; i < 19; ++i) checksum += send_buffer[i];
    //         send_buffer[19] = checksum;

    //         // 전송
    //         ros_ser.write(send_buffer, 20);
    //         // ros_ser.flush();

    //         // 응답 수신
    //         std::vector<unsigned char> recv_buffer;
    //         recv_buffer.reserve(9);
    //         auto start = std::chrono::steady_clock::now();

    //         while (recv_buffer.size() < 9)
    //         {
    //             if (ros_ser.available())
    //             {
    //                 size_t bytes = ros_ser.available();
    //                 std::vector<unsigned char> temp(bytes);
    //                 size_t len = ros_ser.read(temp.data(), bytes);
    //                 recv_buffer.insert(recv_buffer.end(), temp.begin(), temp.begin() + len);
    //             }

    //             auto now = std::chrono::steady_clock::now();
    //             if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
    //             {
    //                 RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "[%s] Timeout waiting for response", context.c_str());
    //                 return false;
    //             }
    //         }

    //         // 응답 확인
    //         if (recv_buffer[1] == 0xEB && recv_buffer[0] == 0x90 && recv_buffer[4] == 0x12)
    //         {
    //             if (recv_buffer[7] == 0x01)
    //             {
    //                 RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "[%s] Command success", context.c_str());
    //                 RCLCPP_INFO(this->get_logger(), "data : %x %x %x %x %x %x %x %x %x ",
    //                 recv_buffer[0], recv_buffer[1], recv_buffer[2], recv_buffer[3], recv_buffer[4], recv_buffer[5],
    //                 recv_buffer[6], recv_buffer[7], recv_buffer[8]);
    //                 return true;
    //             }
    //             else
    //             {
    //                 RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "[%s] Command failed (code: 0x%02X)", context.c_str(), recv_buffer[7]);
    //                 return false;
    //             }
    //         }

    //         RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "[%s] Invalid or malformed response", context.c_str());
    //         return false;
    //     }
    //     catch (const std::exception &e) {
    //         RCLCPP_ERROR(rclcpp::get_logger("HandInterface"), "Serial exception in send_set_one_command: %s", e.what());
    //         return false;
    //     }
    // }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandControlActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}