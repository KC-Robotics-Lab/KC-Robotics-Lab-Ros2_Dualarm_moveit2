#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "rclcpp_action/rclcpp_action.hpp"

// msg
#include "inspire_hand_interface/msg/posact.hpp"
#include "inspire_hand_interface/msg/angleact.hpp"
#include "inspire_hand_interface/msg/forceact.hpp"
#include "inspire_hand_interface/msg/current.hpp"
#include "inspire_hand_interface/msg/error.hpp"
#include "inspire_hand_interface/msg/temp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
// srv
#include "inspire_hand_interface/srv/setangle.hpp"
#include "inspire_hand_interface/srv/getangleact.hpp"
#include "inspire_hand_interface/srv/setpos.hpp"
#include "inspire_hand_interface/srv/setspeed.hpp"
#include "inspire_hand_interface/srv/setforce.hpp"
#include "inspire_hand_interface/srv/getangleset.hpp"
#include "inspire_hand_interface/srv/getposact.hpp"
#include "inspire_hand_interface/srv/getposset.hpp"
#include "inspire_hand_interface/srv/getspeedset.hpp"
#include "inspire_hand_interface/srv/getforceact.hpp"
#include "inspire_hand_interface/srv/getforceset.hpp"
#include "inspire_hand_interface/srv/getcurrentact.hpp"
#include "inspire_hand_interface/srv/geterror.hpp"
#include "inspire_hand_interface/srv/gettemp.hpp"
#include "inspire_hand_interface/srv/gethandid.hpp"
#include "inspire_hand_interface/srv/getbaudrate.hpp"
// action
#include "inspire_hand_interface/action/set_angle.hpp"

#define posset		        0x05C2
#define angleset	        0x05CE
#define forceset	        0x05DA
#define speedset	        0x05F2
#define getpos_act          0x05FE
#define getangle_act        0x060A
#define getforce_act        0x062E
#define getcurrent          0x063A
#define geterror            0x0646
#define gettemp             0x0652

#define inspire_ID          0x01

// serial::Serial ros_ser;
unsigned char send_buffer[64] = {0};
unsigned char recv_buffer[256] = {0};
rclcpp::WallRate loop_rate(25.0);        //40ms
// rclcpp::WallRate loop_rate(33.3);
// rclcpp::WallRate loop_rate(50.0);
std::mutex serial_mutex;

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::placeholders;

using SetAngle = inspire_hand_interface::action::SetAngle;
using GoalHandleSetAngle = rclcpp_action::ServerGoalHandle<SetAngle>;

class Hand_control : public rclcpp::Node
{
    public:
        Hand_control() : Node("getparam_publisher")
        {
            try
            {
                // ros_ser.setPort("/dev/ttyUSB0");
                ros_ser.setPort("/dev/ttyUSB0");
                ros_ser.setBaudrate(115200);
                serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                ros_ser.setTimeout(to);
                ros_ser.open();
                if (ros_ser.isOpen()) {
                    RCLCPP_INFO(this->get_logger(), "Serial port opened.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
                    rclcpp::shutdown();
                    return;
                }
                // ros_ser.flush();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            catch (serial::IOException &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Serial exception: %s", e.what());
                rclcpp::shutdown();
                return;
            }

            Setangle_Server = this->create_service<inspire_hand_interface::srv::Setangle>("Setangle",
                            std::bind(&Hand_control::setangle_callback,this,_1,_2));
            Setpos_Server = this->create_service<inspire_hand_interface::srv::Setpos>("Setpos",
                            std::bind(&Hand_control::setpos_callback,this,_1,_2));
            Setspeed_Server = this->create_service<inspire_hand_interface::srv::Setspeed>("Setspeed",
                            std::bind(&Hand_control::setspeed_callback,this,_1,_2));
            Setforce_Server = this->create_service<inspire_hand_interface::srv::Setforce>("Setforce",
                            std::bind(&Hand_control::setforce_callback,this,_1,_2));
            Getangleset_Server = this->create_service<inspire_hand_interface::srv::Getangleset>("Getangleset",
                            std::bind(&Hand_control::getangleset_callback, this, _1, _2));
            Getposset_Server = this->create_service<inspire_hand_interface::srv::Getposset>("Getposset",
                            std::bind(&Hand_control::getposset_callback, this, _1, _2));
            Getspeedset_Server = this->create_service<inspire_hand_interface::srv::Getspeedset>("Getspeedset",
                            std::bind(&Hand_control::getspeedset_callback, this, _1, _2));
            Getforceset_Server = this->create_service<inspire_hand_interface::srv::Getforceset>("Getforceset",
                            std::bind(&Hand_control::getforceset_callback, this, _1, _2));
            Getangleact_Server = this->create_service<inspire_hand_interface::srv::Getangleact>("Getangleact",
                            std::bind(&Hand_control::getangleact_callback, this, _1, _2));
            Getposact_Server = this->create_service<inspire_hand_interface::srv::Getposact>("Getposact",
                            std::bind(&Hand_control::getposact_callback, this, _1, _2));
            Getforceact_Server = this->create_service<inspire_hand_interface::srv::Getforceact>("Getforceact",
                            std::bind(&Hand_control::getforceact_callback, this, _1, _2));
            Getcurrentact_Server = this->create_service<inspire_hand_interface::srv::Getcurrentact>("Getcurrentact",
                            std::bind(&Hand_control::getcurrentact_callback, this, _1, _2));
            Geterror_Server = this->create_service<inspire_hand_interface::srv::Geterror>("Geterror",
                            std::bind(&Hand_control::geterror_callback, this, _1, _2));
            Gettemp_Server = this->create_service<inspire_hand_interface::srv::Gettemp>("Gettemp",
                            std::bind(&Hand_control::gettemp_callback, this, _1, _2));
            Hand_id_Server = this->create_service<inspire_hand_interface::srv::Gethandid>("handid",
                            std::bind(&Hand_control::id_callback, this, _1, _2));
            Baudrate_Server = this->create_service<inspire_hand_interface::srv::Getbaudrate>("baudrate",
                            std::bind(&Hand_control::baud_callback, this, _1, _2));

            // 토픽 생성
            get_posact = this->create_publisher<inspire_hand_interface::msg::Posact>("hand_posact", 10);
            get_angleact = this->create_publisher<inspire_hand_interface::msg::Angleact>("hand_angleact", 10);
            get_forceact = this->create_publisher<inspire_hand_interface::msg::Forceact>("hand_forceact", 10);
            get_current = this->create_publisher<inspire_hand_interface::msg::Current>("hand_current", 10);
            get_error = this->create_publisher<inspire_hand_interface::msg::Error>("hand_error", 10);
            get_temp = this->create_publisher<inspire_hand_interface::msg::Temp>("hand_temp", 10);

            little_pub_tip_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/littletip", 10);
            little_pub_nail_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/littlenail", 10);
            little_pub_pad_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/littlepad", 10);

            ring_pub_tip_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/ringtip", 10);
            ring_pub_nail_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/ringnail", 10);
            ring_pub_pad_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/ringpad", 10);

            middle_pub_tip_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/middletip", 10);
            middle_pub_nail_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/middlenail", 10);
            middle_pub_pad_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/middlepad", 10);

            index_pub_tip_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/indextip", 10);
            index_pub_nail_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/indexnail", 10);
            index_pub_pad_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/indexpad", 10);

            thumb_pub_tip_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/thumbtip", 10);
            thumb_pub_nail_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/thumbnail", 10);
            thumb_pub_middle_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/thumbmiddle", 10);
            thumb_pub_pad_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/thumbpad", 10);

            pub_palm_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/palm", 10);

            timer1_ = this->create_wall_timer(
                std::chrono::milliseconds(1),
                std::bind(&Hand_control::publish_message, this)
            );

            sub_angle = this->create_subscription<inspire_hand_interface::msg::Angleact>(
                "hand_angleact", 10,
                std::bind(&Hand_control::get_hand_angleact_callback, this, std::placeholders::_1)
            );

            action_angle_server = rclcpp_action::create_server<SetAngle>(
                this,
                "set_angle",
                std::bind(&Hand_control::angle_handle_goal, this, _1, _2),
                std::bind(&Hand_control::angle_handle_cancel, this, _1),
                std::bind(&Hand_control::angle_handle_accepted, this, _1)
            );

            Hand_init();
        }

        void Hand_init()
        {
            send_set_one_command(inspire_ID, 1, 0x03E8);                             //Hand id
            send_set_one_command(inspire_ID, 0, 0x03EA);                             //baudrate
            send_set_one_command(inspire_ID, 1, 0x03EC);                             //error clear
            send_set_one_command(inspire_ID, 1, 0x03F1);                             //force cal
            std::array<uint16_t, 6> Dspeed = {1000, 1000, 1000, 1000, 1000, 1000};
            send_set_command(inspire_ID, Dspeed, 0x05F2, "default_speed");
            std::array<uint16_t, 6> Dforce = {1000, 1000, 1000, 1000, 1000, 1000};
            send_set_command(inspire_ID, Dforce, 0x05DA, "default_force");
            // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "1111111111");
        }

    private:
        rclcpp::TimerBase::SharedPtr reconnect_timer_;

        serial::Serial ros_ser;
        std::string port_;
        int baudrate_;
        bool hand_initialized_ = false;

        // 서비스 서버 선언
        rclcpp::Service<inspire_hand_interface::srv::Setangle>::SharedPtr Setangle_Server;
        rclcpp::Service<inspire_hand_interface::srv::Setpos>::SharedPtr Setpos_Server;
        rclcpp::Service<inspire_hand_interface::srv::Setspeed>::SharedPtr Setspeed_Server;
        rclcpp::Service<inspire_hand_interface::srv::Setforce>::SharedPtr Setforce_Server;
        rclcpp::Service<inspire_hand_interface::srv::Getangleact>::SharedPtr Getangleact_Server;
        rclcpp::Service<inspire_hand_interface::srv::Getangleset>::SharedPtr Getangleset_Server;
        rclcpp::Service<inspire_hand_interface::srv::Getposact>::SharedPtr Getposact_Server;
        rclcpp::Service<inspire_hand_interface::srv::Getposset>::SharedPtr Getposset_Server;
        rclcpp::Service<inspire_hand_interface::srv::Getspeedset>::SharedPtr Getspeedset_Server;
        rclcpp::Service<inspire_hand_interface::srv::Getforceact>::SharedPtr Getforceact_Server;
        rclcpp::Service<inspire_hand_interface::srv::Getforceset>::SharedPtr Getforceset_Server;
        rclcpp::Service<inspire_hand_interface::srv::Getcurrentact>::SharedPtr Getcurrentact_Server;
        rclcpp::Service<inspire_hand_interface::srv::Geterror>::SharedPtr Geterror_Server;
        rclcpp::Service<inspire_hand_interface::srv::Gettemp>::SharedPtr Gettemp_Server;
        rclcpp::Service<inspire_hand_interface::srv::Gethandid>::SharedPtr Hand_id_Server;
        rclcpp::Service<inspire_hand_interface::srv::Getbaudrate>::SharedPtr Baudrate_Server;

        rclcpp::Subscription<inspire_hand_interface::msg::Angleact>::SharedPtr sub_angle;

        rclcpp_action::Server<SetAngle>::SharedPtr action_angle_server;

        uint16_t angle_current[6] = {0};

        bool send_set_one_command(uint8_t hand_id, uint16_t value, uint16_t reg)
        {
            std::lock_guard<std::mutex> lock(serial_mutex);


            if (!ros_ser.isOpen()) {
                RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "Serial port not open. Skipping send_set_one_command.");
                return false;
            }

            try
            {
                // 버퍼 비우기
                // while (ros_ser.available())
                // {
                //     std::vector<unsigned char> flush_buf(ros_ser.available());
                //     ros_ser.read(flush_buf.data(), flush_buf.size());
                // }

                // 패킷 작성
                uint8_t checksum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = hand_id;
                send_buffer[3] = 0x05;
                send_buffer[4] = 0x12;
                send_buffer[5] = reg & 0xFF;
                send_buffer[6] = reg >> 8;
                send_buffer[7] = value & 0xFF;
                send_buffer[8] = (value >> 8) & 0xFF;

                for (int i = 2; i < 9; ++i) checksum += send_buffer[i];
                send_buffer[9] = checksum;

                // 전송
                ros_ser.write(send_buffer, 10);
                // ros_ser.flush();

                // 응답 수신
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(9);
                auto start = std::chrono::steady_clock::now();

                while (recv_buffer.size() < 9)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes = ros_ser.available();
                        std::vector<unsigned char> temp(bytes);
                        size_t len = ros_ser.read(temp.data(), bytes);
                        recv_buffer.insert(recv_buffer.end(), temp.begin(), temp.begin() + len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "Timeout waiting for response");
                        return false;
                    }
                }

                // 응답 확인
                if (recv_buffer[1] == 0xEB && recv_buffer[0] == 0x90 && recv_buffer[4] == 0x12)
                {
                    if (recv_buffer[7] == 0x01)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "Command success");
                        RCLCPP_INFO(this->get_logger(), "data : %x %x %x %x %x %x %x %x %x",
                                        recv_buffer[0], recv_buffer[1], recv_buffer[2], recv_buffer[3], recv_buffer[4], recv_buffer[5],
                                        recv_buffer[6], recv_buffer[7], recv_buffer[8]);
                        return true;
                    }
                    else
                    {
                        RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "Command failed (code: 0x%02X)", recv_buffer[7]);
                        return false;
                    }
                }

                RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "Invalid or malformed response");
                return false;
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("HandInterface"), "Serial exception in send_set_one_command: %s", e.what());
                return false;
            }
        }

        bool send_set_command(uint8_t hand_id, const std::array<uint16_t, 6>& values, uint16_t reg, const std::string& context)
        {
            std::lock_guard<std::mutex> lock(serial_mutex);

            if (!ros_ser.isOpen()) {
                RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "Serial port not open. Skipping send_set_one_command.");
                return false;
            }

            try
            {
                // 버퍼 비우기
                while (ros_ser.available())
                {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                // 패킷 작성
                uint8_t checksum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = hand_id;
                send_buffer[3] = 0x0F;
                send_buffer[4] = 0x12;
                send_buffer[5] = reg & 0xFF;
                send_buffer[6] = reg >> 8;
                for (size_t i = 0; i < 6; ++i)
                {
                    send_buffer[7 + i * 2] = values[i] & 0xFF;
                    send_buffer[8 + i * 2] = (values[i] >> 8) & 0xFF;
                }
                for (int i = 2; i < 19; ++i) checksum += send_buffer[i];
                send_buffer[19] = checksum;

                // 전송
                ros_ser.write(send_buffer, 20);
                // ros_ser.flush();

                // 응답 수신
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(9);
                auto start = std::chrono::steady_clock::now();

                while (recv_buffer.size() < 9)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes = ros_ser.available();
                        std::vector<unsigned char> temp(bytes);
                        size_t len = ros_ser.read(temp.data(), bytes);
                        recv_buffer.insert(recv_buffer.end(), temp.begin(), temp.begin() + len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "[%s] Timeout waiting for response", context.c_str());
                        return false;
                    }
                }

                // 응답 확인
                if (recv_buffer[1] == 0xEB && recv_buffer[0] == 0x90 && recv_buffer[4] == 0x12)
                {
                    if (recv_buffer[7] == 0x01)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "[%s] Command success", context.c_str());
                        RCLCPP_INFO(this->get_logger(), "data : %x %x %x %x %x %x %x %x %x ",
                        recv_buffer[0], recv_buffer[1], recv_buffer[2], recv_buffer[3], recv_buffer[4], recv_buffer[5],
                        recv_buffer[6], recv_buffer[7], recv_buffer[8]);
                        return true;
                    }
                    else
                    {
                        RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "[%s] Command failed (code: 0x%02X)", context.c_str(), recv_buffer[7]);
                        return false;
                    }
                }

                RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "[%s] Invalid or malformed response", context.c_str());
                return false;
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("HandInterface"), "Serial exception in send_set_one_command: %s", e.what());
                return false;
            }
        }

        bool request_hand_data(uint8_t hand_id, uint16_t reg, std::array<uint16_t, 6>&output, const std::string &log_label)
        {
            std::lock_guard<std::mutex> lock(serial_mutex);

            if (!ros_ser.isOpen()) {
                RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "Serial port not open. Skipping send_set_one_command.");
                return false;
            }

            try
            {
                // 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = reg & 0xFF;
                send_buffer[6] = reg >> 8;
                send_buffer[7] = 0x0C;
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                // ros_ser.flush();

                // 응답 수신
                size_t expected_size = 7 + 12 + 1;
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size) {
                    if (ros_ser.available()) {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100) {
                        RCLCPP_WARN(this->get_logger(), "[%s] Timeout reading serial response", log_label.c_str());
                        return false;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() >= expected_size && recv_buffer[1] == 0xEB &&
                    recv_buffer[0] == 0x90 && recv_buffer[4] == 0x11)
                {
                    try {
                        // output.resize(6);
                        for (size_t i = 0; i < 6; ++i) {
                            uint16_t val = (recv_buffer[7 + i * 2] & 0xFF) | ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
                            output[i] = static_cast<int32_t>(val);
                        }
                        return true;
                    }
                    catch (const std::exception &e) {
                        RCLCPP_ERROR(this->get_logger(), "[%s] Exception during parsing: %s", log_label.c_str(), e.what());
                        return false;
                    }
                }

                RCLCPP_WARN(this->get_logger(), "[%s] Invalid or no response received.", log_label.c_str());
                return false;
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("HandInterface"), "Serial exception in send_set_one_command: %s", e.what());
                return false;
            }
        }

        bool request_hand_one_data(uint8_t hand_id, uint16_t reg, uint8_t &output_byte, const std::string &log_label)
        {
            std::lock_guard<std::mutex> lock(serial_mutex);

            if (!ros_ser.isOpen()) {
                RCLCPP_WARN(rclcpp::get_logger("HandInterface"), "Serial port not open. Skipping send_set_one_command.");
                return false;
            }

            try
            {
                // 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    uint8_t dummy;
                    ros_ser.read(&dummy, 1);
                }

                // 요청 패킷 생성 (1바이트 요청)
                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = reg & 0xFF;
                send_buffer[6] = reg >> 8;
                send_buffer[7] = 0x01;  // 읽을 바이트 수 (1바이트만)
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                // ros_ser.flush();

                // 응답 수신 (1바이트만 포함된 응답 → 총 길이: 7(header) + 1(data) + 1(checksum) = 9)
                const size_t expected_size = 9;
                std::vector<uint8_t> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size) {
                    if (ros_ser.available()) {
                        uint8_t byte;
                        ros_ser.read(&byte, 1);
                        recv_buffer.push_back(byte);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100) {
                        RCLCPP_WARN(this->get_logger(), "[%s] Timeout reading serial response", log_label.c_str());
                        return false;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() == expected_size && recv_buffer[1] == 0xEB &&
                    recv_buffer[0] == 0x90 && recv_buffer[4] == 0x11)
                {
                    output_byte = recv_buffer[7];  // 1바이트 데이터 위치
                    return true;
                }

                RCLCPP_WARN(this->get_logger(), "[%s] Invalid or malformed response", log_label.c_str());
                return false;
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("HandInterface"), "Serial exception in send_set_one_command: %s", e.what());
                return false;
            }
        }

        void id_callback(const inspire_hand_interface::srv::Gethandid::Request::SharedPtr request,
            const inspire_hand_interface::srv::Gethandid::Response::SharedPtr response)
        {
            uint8_t id;
            (void)request;
            if(request_hand_one_data(1, 0x03E8, id, "hand_id"))
            {
                // RCLCPP_INFO(this->get_logger(), "id : %d", id);
                response->id = id;
            }
        }

        void baud_callback(const inspire_hand_interface::srv::Getbaudrate::Request::SharedPtr request,
            const inspire_hand_interface::srv::Getbaudrate::Response::SharedPtr response)
        {
            uint8_t baudrate;
            (void)request;
            if(request_hand_one_data(1, 0x03EA, baudrate, "baudrate"))
            {
                // RCLCPP_INFO(this->get_logger(), "baudrate : %d", baudrate);
                response->baudrate = baudrate;
            }
        }

        void setpos_callback(const inspire_hand_interface::srv::Setpos::Request::SharedPtr request,
            const inspire_hand_interface::srv::Setpos::Response::SharedPtr response)
        {
            if (request->status == "set_pos") {
                RCLCPP_INFO(this->get_logger(), "status: set_pos");
                std::array<uint16_t, 6> poses = {
                    request->pos0, request->pos1, request->pos2,
                    request->pos3, request->pos4, request->pos5
                };
                response->pos_accepted = send_set_command(request->hand_id, poses, posset, "set_pos");
            } else {
                RCLCPP_INFO(this->get_logger(), "Invalid status: %s", request->status.c_str());
                response->pos_accepted = false;
            }
        }

        void setangle_callback(const inspire_hand_interface::srv::Setangle::Request::SharedPtr request,
            const inspire_hand_interface::srv::Setangle::Response::SharedPtr response)
        {
            if (request->status == "set_angle") {
                RCLCPP_INFO(this->get_logger(), "status: set_angle");
                std::array<uint16_t, 6> angles = {
                    request->angle0, request->angle1, request->angle2,
                    request->angle3, request->angle4, request->angle5
                };
                response->angle_accepted = send_set_command(request->hand_id, angles, angleset, "set_angle");
            } else {
                RCLCPP_INFO(this->get_logger(), "Invalid status: %s", request->status.c_str());
                response->angle_accepted = false;
            }
        }

        void setforce_callback(const inspire_hand_interface::srv::Setforce::Request::SharedPtr request,
            const inspire_hand_interface::srv::Setforce::Response::SharedPtr response)
        {
            if (request->status == "set_force") {
                RCLCPP_INFO(this->get_logger(), "status: set_force");
                std::array<uint16_t, 6> poses = {
                    request->force0, request->force1, request->force2,
                    request->force3, request->force4, request->force5
                };
                response->force_accepted = send_set_command(request->hand_id, poses, forceset, "set_force");
            } else {
                RCLCPP_INFO(this->get_logger(), "Invalid status: %s", request->status.c_str());
                response->force_accepted = false;
            }
        }

        void setspeed_callback(const inspire_hand_interface::srv::Setspeed::Request::SharedPtr request,
            const inspire_hand_interface::srv::Setspeed::Response::SharedPtr response)
        {
            if (request->status == "set_speed") {
                RCLCPP_INFO(this->get_logger(), "status: set_speed");
                std::array<uint16_t, 6> angles = {
                    request->speed0, request->speed1, request->speed2,
                    request->speed3, request->speed4, request->speed5
                };
                response->speed_accepted = send_set_command(request->hand_id, angles, speedset, "set_speed");
            } else {
                RCLCPP_INFO(this->get_logger(), "Invalid status: %s", request->status.c_str());
                response->speed_accepted = false;
            }
        }

        void getposset_callback(const inspire_hand_interface::srv::Getposset::Request::SharedPtr request,
            const inspire_hand_interface::srv::Getposset::Response::SharedPtr response)
        {
            if (request->status == "get_posset")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                if (request_hand_data(request->hand_id, posset, response->curposset, "POS_SET"))
                {
                    RCLCPP_INFO(this->get_logger(), "pos set : %d %d %d %d %d %d",
                            response->curposset[0], response->curposset[1], response->curposset[2],
                            response->curposset[3], response->curposset[4], response->curposset[5]);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_posset 아님: %s", request->status.c_str());
            }
        }

        void getangleset_callback(const inspire_hand_interface::srv::Getangleset::Request::SharedPtr request,
                    const inspire_hand_interface::srv::Getangleset::Response::SharedPtr response)
        {
            if (request->status == "get_angleset")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                if (request_hand_data(request->hand_id, angleset, response->curangleset, "ANGLE_SET"))
                {
                    RCLCPP_INFO(this->get_logger(), "angle set : %d %d %d %d %d %d",
                            response->curangleset[0], response->curangleset[1], response->curangleset[2],
                            response->curangleset[3], response->curangleset[4], response->curangleset[5]);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleset가 아님: %s", request->status.c_str());
            }
        }

        void getforceset_callback(const inspire_hand_interface::srv::Getforceset::Request::SharedPtr request,
            const inspire_hand_interface::srv::Getforceset::Response::SharedPtr response)
        {
            if (request->status == "get_forceset")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                if (request_hand_data(request->hand_id, forceset, response->curforceset, "FORCE_SET"))
                {
                    RCLCPP_INFO(this->get_logger(), "force set : %d %d %d %d %d %d",
                            response->curforceset[0], response->curforceset[1], response->curforceset[2],
                            response->curforceset[3], response->curforceset[4], response->curforceset[5]);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_forceset 아님: %s", request->status.c_str());
            }
        }

        void getspeedset_callback(const inspire_hand_interface::srv::Getspeedset::Request::SharedPtr request,
            const inspire_hand_interface::srv::Getspeedset::Response::SharedPtr response)
        {
            if (request->status == "get_speedset")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                if (request_hand_data(request->hand_id, speedset, response->curspeedset, "SPEED_SET"))
                {
                    RCLCPP_INFO(this->get_logger(), "speed set : %d %d %d %d %d %d",
                            response->curspeedset[0], response->curspeedset[1], response->curspeedset[2],
                            response->curspeedset[3], response->curspeedset[4], response->curspeedset[5]);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_speedset가 아님: %s", request->status.c_str());
            }
        }

        void getposact_callback(const inspire_hand_interface::srv::Getposact::Request::SharedPtr request,
            const inspire_hand_interface::srv::Getposact::Response::SharedPtr response)
        {
            if (request->status == "get_posact")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                if (request_hand_data(request->hand_id, getpos_act, response->curposact, "POS_ACT"))
                {
                    RCLCPP_INFO(this->get_logger(), "pos act : %d %d %d %d %d %d",
                            response->curposact[0], response->curposact[1], response->curposact[2],
                            response->curposact[3], response->curposact[4], response->curposact[5]);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_posact 아님: %s", request->status.c_str());
            }
        }

        void getangleact_callback(const inspire_hand_interface::srv::Getangleact::Request::SharedPtr request,
            const inspire_hand_interface::srv::Getangleact::Response::SharedPtr response)
        {
            if (request->status == "get_angleact")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                if (request_hand_data(request->hand_id, getangle_act, response->curangleact, "ANGLE_ACT"))
                {
                    RCLCPP_INFO(this->get_logger(), "angle act : %d %d %d %d %d %d",
                            response->curangleact[0], response->curangleact[1], response->curangleact[2],
                            response->curangleact[3], response->curangleact[4], response->curangleact[5]);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleact 아님: %s", request->status.c_str());
            }
        }

        void getforceact_callback(const inspire_hand_interface::srv::Getforceact::Request::SharedPtr request,
            const inspire_hand_interface::srv::Getforceact::Response::SharedPtr response)
        {
            if (request->status == "get_forceact")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                if (request_hand_data(request->hand_id, getforce_act, response->curforceact, "FORCE_ACT"))
                {
                    RCLCPP_INFO(this->get_logger(), "force act : %d %d %d %d %d %d",
                            response->curforceact[0], response->curforceact[1], response->curforceact[2],
                            response->curforceact[3], response->curforceact[4], response->curforceact[5]);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_forceact 아님: %s", request->status.c_str());
            }
        }

        void getcurrentact_callback(const inspire_hand_interface::srv::Getcurrentact::Request::SharedPtr request,
            const inspire_hand_interface::srv::Getcurrentact::Response::SharedPtr response)
        {
            if (request->status == "get_currentact")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                if (request_hand_data(request->hand_id, getcurrent, response->curcurrent, "CURRENT_ACT"))
                {
                    RCLCPP_INFO(this->get_logger(), "current act : %d %d %d %d %d %d",
                            response->curcurrent[0], response->curcurrent[1], response->curcurrent[2],
                            response->curcurrent[3], response->curcurrent[4], response->curcurrent[5]);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_currentact 아님: %s", request->status.c_str());
            }
        }

        void geterror_callback(const inspire_hand_interface::srv::Geterror::Request::SharedPtr request,
            const inspire_hand_interface::srv::Geterror::Response::SharedPtr response)
        {
            if (request->status == "get_error")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                if (request_hand_data(request->hand_id, geterror, response->error, "ERROR_ACT"))
                {
                    RCLCPP_INFO(this->get_logger(), "error : %d %d %d %d %d %d",
                            response->error[0], response->error[1], response->error[2],
                            response->error[3], response->error[4], response->error[5]);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_error 아님: %s", request->status.c_str());
            }
        }

        void gettemp_callback(const inspire_hand_interface::srv::Gettemp::Request::SharedPtr request,
            const inspire_hand_interface::srv::Gettemp::Response::SharedPtr response)
        {
            if (request->status == "get_temp")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                if (request_hand_data(request->hand_id, gettemp, response->temp, "TEMP_ACT"))
                {
                    RCLCPP_INFO(this->get_logger(), "temp : %d %d %d %d %d %d",
                            response->temp[0], response->temp[1], response->temp[2],
                            response->temp[3], response->temp[4], response->temp[5]);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_temp 아님: %s", request->status.c_str());
            }
        }

        std::vector<uint16_t> parse_packets(const std::vector<unsigned char>& buffer, uint8_t len)
        {
            std::vector<uint16_t> data;

            // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "aaaaaaaaaaaaaaaa");
            // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "buffer.size: %d", buffer.size());

            for (size_t offset = 0; offset + 8 <= buffer.size(); ++offset)
            {
                // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "bbbbbbbbbbbbbbb");

                // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "buffer[%zu]: 0x%02X", offset, static_cast<uint16_t>(buffer[offset]));
                if (buffer[offset] == 0x90 && buffer[offset + 1] == 0xEB && buffer[offset + 4] == 0x11)
                {
                    // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "cccccccccccccccccc");
                    // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "cond: [%02X %02X %02X]", buffer[offset], buffer[offset + 1], buffer[offset + 4]);
                    size_t total_len = 7 + len + 1;
                    // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "tot: %d", total_len );

                    if (offset + total_len > buffer.size())
                    {
                        // 패킷이 부족함
                        // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "off+tot: %d", offset + total_len );
                        // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "buffer.size: %d", buffer.size() );
                        continue;
                    }

                    // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "eeeeeeeeeeeeeee");

                    for (size_t i = 0; i < len/2; ++i)
                    {
                        // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "fffffffffffffffff");
                        size_t low_index = offset + 7 + i * 2;
                        size_t high_index = offset + 8 + i * 2;
                        uint16_t value = buffer[low_index] | (buffer[high_index] << 8);
                        data.push_back(value);
                    }

                    // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "ffffffffffffff");

                    return data;  // 유효한 패킷 파싱 완료
                }
            }

            return {};  // 유효한 패킷 없음
        }

        std::vector<uint8_t> parse_packet(const std::vector<unsigned char>& buffer, uint8_t len)
        {
            std::vector<uint8_t> data;

            for (size_t offset = 0; offset + 8 <= buffer.size(); ++offset)
            {
                if (buffer[offset] == 0x90 && buffer[offset + 1] == 0xEB && buffer[offset + 4] == 0x11)
                {
                    size_t total_len = 7 + len + 1;

                    if (offset + total_len > buffer.size())
                    {
                        continue;
                    }

                    for (size_t i = 0; i < 6; ++i)
                    {
                        size_t low_index = offset + 7 + i;
                        uint8_t value = buffer[low_index];
                        data.push_back(value);
                    }

                    return data;  // 유효한 패킷 파싱 완료
                }
            }

            return {};  // 유효한 패킷 없음
        }

        std::vector<uint8_t> read_register(uint8_t hand_id, uint16_t reg, uint8_t len)
        {
            std::lock_guard<std::mutex> lock(serial_mutex);
            std::memset(recv_buffer, 0, sizeof(recv_buffer));

            if (!ros_ser.isOpen()) {
                RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
                return {};
              }
            //   else RCLCPP_INFO(this->get_logger(), "Serial port is open");

            try
            {
                uint8_t checksum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = reg & 0xFF;
                send_buffer[6] = reg >> 8;
                send_buffer[7] = len;
                for (int i = 2; i < 8; i++) checksum += send_buffer[i];
                send_buffer[8] = checksum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();

                size_t expected_size = 7 + len + 1;
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);  // 여유를 둠
                // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "expected_size : %d", expected_size);

                auto start = std::chrono::steady_clock::now();
                while (std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - start).count() < 50)
                {
                    if (ros_ser.available())
                    {
                        size_t to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buf(to_read);
                        size_t read_len = ros_ser.read(temp_buf.data(), to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buf.begin(), temp_buf.begin() + read_len);

                        auto parsed = parse_packet(recv_buffer, len);
                        if (!parsed.empty())
                        {
                            return parsed;
                        }
                    }
                }

                RCLCPP_WARN(this->get_logger(), "Timeout or invalid packet during read_registers");
                return {};
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("HandInterface"), "Serial exception in read_registers: %s", e.what());
                return {};
            }
        }


        std::vector<uint16_t> read_registers(uint8_t hand_id, uint16_t reg, uint8_t len)
        {
            std::lock_guard<std::mutex> lock(serial_mutex);
            std::memset(recv_buffer, 0, sizeof(recv_buffer));

            if (!ros_ser.isOpen()) {
                RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
                return {};
              }
            //   else RCLCPP_INFO(this->get_logger(), "Serial port is open");

            try
            {
                uint8_t checksum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = reg & 0xFF;
                send_buffer[6] = reg >> 8;
                send_buffer[7] = len;
                for (int i = 2; i < 8; i++) checksum += send_buffer[i];
                send_buffer[8] = checksum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();

                size_t expected_size = 7 + len + 1;
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);  // 여유를 둠
                // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "expected_size : %d", expected_size);

                auto start = std::chrono::steady_clock::now();
                while (std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - start).count() < 50)
                {
                    if (ros_ser.available())
                    {
                        // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "available : %d", ros_ser.available());
                        // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "2222222");
                        size_t to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buf(to_read);
                        size_t read_len = ros_ser.read(temp_buf.data(), to_read);
                        // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "read_len : %d", read_len);
                        recv_buffer.insert(recv_buffer.end(), temp_buf.begin(), temp_buf.begin() + read_len);

                        auto parsed = parse_packets(recv_buffer, len);
                        if (!parsed.empty())
                        {
                            // RCLCPP_INFO(rclcpp::get_logger("HandInterface"), "333333");
                            return parsed;
                        }
                    }
                }

                RCLCPP_WARN(this->get_logger(), "Timeout or invalid packet during read_registers");
                return {};
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("HandInterface"), "Serial exception in read_registers: %s", e.what());
                return {};
            }
        }

        void publish_message()
        {
            auto posact_msg = inspire_hand_interface::msg::Posact();
            auto angleact_msg = inspire_hand_interface::msg::Angleact();
            auto forceact_msg = inspire_hand_interface::msg::Forceact();
            auto current_msg = inspire_hand_interface::msg::Current();
            auto error_msg = inspire_hand_interface::msg::Error();
            auto temp_msg = inspire_hand_interface::msg::Temp();

            std_msgs::msg::UInt16MultiArray little_tip_msg;
            std_msgs::msg::UInt16MultiArray little_nail_msg;
            std_msgs::msg::UInt16MultiArray little_pad_msg;

            std_msgs::msg::UInt16MultiArray ring_tip_msg;
            std_msgs::msg::UInt16MultiArray ring_nail_msg;
            std_msgs::msg::UInt16MultiArray ring_pad_msg;

            std_msgs::msg::UInt16MultiArray middle_tip_msg;
            std_msgs::msg::UInt16MultiArray middle_nail_msg;
            std_msgs::msg::UInt16MultiArray middle_pad_msg;

            std_msgs::msg::UInt16MultiArray index_tip_msg;
            std_msgs::msg::UInt16MultiArray index_nail_msg;
            std_msgs::msg::UInt16MultiArray index_pad_msg;

            std_msgs::msg::UInt16MultiArray thumb_tip_msg;
            std_msgs::msg::UInt16MultiArray thumb_nail_msg;
            std_msgs::msg::UInt16MultiArray thumb_middle_msg;
            std_msgs::msg::UInt16MultiArray thumb_pad_msg;

            std_msgs::msg::UInt16MultiArray palm_msg;

            auto little_pad_msgtip_data = read_registers(1, 0x0BB8, 18);
            auto little_pad_msgnail_data = read_registers(1, 0x0BCA, 192);
            auto little_pad_msgpad_data = read_registers(1, 0x0C8A, 160);
            little_tip_msg.data.assign(little_pad_msgtip_data.begin(), little_pad_msgtip_data.end());
            little_nail_msg.data.assign(little_pad_msgnail_data.begin(), little_pad_msgnail_data.end());
            little_pad_msg.data.assign(little_pad_msgpad_data.begin(), little_pad_msgpad_data.end());

            auto ring_tip_data = read_registers(1, 0x0D2A, 18);
            auto ring_nail_data = read_registers(1, 0x0D3C, 192);
            auto ring_pad_data = read_registers(1, 0x0DFC, 160);
            ring_tip_msg.data.assign(ring_tip_data.begin(), ring_tip_data.end());
            ring_nail_msg.data.assign(ring_nail_data.begin(), ring_nail_data.end());
            ring_pad_msg.data.assign(ring_pad_data.begin(), ring_pad_data.end());

            auto middle_tip_data = read_registers(1, 0x0E9C, 18);
            auto middle_nail_data = read_registers(1, 0x0EAE, 192);
            auto middle_pad_data = read_registers(1, 0x0F6E, 160);
            middle_tip_msg.data.assign(middle_tip_data.begin(), middle_tip_data.end());
            middle_nail_msg.data.assign(middle_nail_data.begin(), middle_nail_data.end());
            middle_pad_msg.data.assign(middle_pad_data.begin(), middle_pad_data.end());

            auto index_tip_data  = read_registers(1, 0x100E, 18);
            auto index_nail_data = read_registers(1, 0x1020, 192);
            auto index_pad_data  = read_registers(1, 0x10E0, 160);
            index_tip_msg.data.assign(index_tip_data.begin(), index_tip_data.end());
            index_nail_msg.data.assign(index_nail_data.begin(), index_nail_data.end());
            index_pad_msg.data.assign(index_pad_data.begin(), index_pad_data.end());

            auto thumb_tip_data = read_registers(1, 0x1180, 18);
            auto thumb_nail_data = read_registers(1, 0x1192, 192);
            auto thumb_middle_data = read_registers(1, 0x1252, 18);
            auto thumb_pad_data = read_registers(1, 0x1264, 192);
            thumb_tip_msg.data.assign(thumb_tip_data.begin(), thumb_tip_data.end());
            thumb_nail_msg.data.assign(thumb_nail_data.begin(), thumb_nail_data.end());
            thumb_middle_msg.data.assign(thumb_middle_data.begin(), thumb_middle_data.end());
            thumb_pad_msg.data.assign(thumb_pad_data.begin(), thumb_pad_data.end());

            auto palm_data = read_registers(1, 0x1324, 224);
            palm_msg.data.assign(palm_data.begin(), palm_data.end());


            auto pos_ = read_registers(1, getpos_act, 12);
            for (int i = 0; i < 6; i++)
                posact_msg.pos[i] = pos_[i];
            get_posact->publish(posact_msg);

            auto angle_ = read_registers(1, getangle_act, 12);
            for (int i = 0; i < 6; i++)
                angleact_msg.angle[i] = angle_[i];
            get_angleact->publish(angleact_msg);

            auto force_ = read_registers(1, getforce_act, 12);
            for (int i = 0; i < 6; i++)
                forceact_msg.force[i] = force_[i];
            get_forceact->publish(forceact_msg);

            auto current_ = read_registers(1, getcurrent, 12);
            for (int i = 0; i < 6; i++)
                current_msg.current[i] = current_[i];
            get_current->publish(current_msg);

            auto error_ = read_register(1, geterror, 6);
            for (int i = 0; i < 6; i++)
                error_msg.error[i] = error_[i];
            get_error->publish(error_msg);

            auto temp_ = read_register(1, gettemp, 6);
            for (int i = 0; i < 6; i++)
                temp_msg.temp[i] = temp_[i];
            get_temp->publish(temp_msg);

            little_pub_tip_->publish(little_tip_msg);
            little_pub_nail_->publish(little_nail_msg);
            little_pub_pad_->publish(little_pad_msg);

            ring_pub_tip_->publish(ring_tip_msg);
            ring_pub_nail_->publish(ring_nail_msg);
            ring_pub_pad_->publish(ring_pad_msg);

            middle_pub_tip_->publish(middle_tip_msg);
            middle_pub_nail_->publish(middle_nail_msg);
            middle_pub_pad_->publish(middle_pad_msg);

            index_pub_tip_->publish(index_tip_msg);
            index_pub_nail_->publish(index_nail_msg);
            index_pub_pad_->publish(index_pad_msg);

            thumb_pub_tip_->publish(thumb_tip_msg);
            thumb_pub_nail_->publish(thumb_nail_msg);
            thumb_pub_middle_->publish(thumb_middle_msg);
            thumb_pub_pad_->publish(thumb_pad_msg);

            pub_palm_->publish(palm_msg);
        }

        void get_hand_angleact_callback(const inspire_hand_interface::msg::Angleact::SharedPtr msg)
        {
            for(int i=0; i<6; i++)
            {
                angle_current[i] = msg->angle[i];
                // RCLCPP_INFO(this->get_logger(), "msg->angle[%d] : %d", i, angle_current[i]);
                // RCLCPP_INFO(this->get_logger(), "angle[%d] : %d", i, angle_current[i]);
            }
        }

        // goal 수락
        rclcpp_action::GoalResponse angle_handle_goal(const rclcpp_action::GoalUUID &,
            std::shared_ptr<const SetAngle::Goal> goal)
        {
            // 유효성 체크
            for (int i=0; i<6; i++)
            {
                uint16_t angle = 0;
                switch(i) {
                    case 0: angle = goal->angle0; break;
                    case 1: angle = goal->angle1; break;
                    case 2: angle = goal->angle2; break;
                    case 3: angle = goal->angle3; break;
                    case 4: angle = goal->angle4; break;
                    case 5: angle = goal->angle5; break;
                }

                if (angle > 1000) {
                    RCLCPP_WARN(this->get_logger(), "Goal rejected: angle[%d]=%d is out of range", i, angle);
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Goal accepted: angles are valid");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // 취소
        rclcpp_action::CancelResponse angle_handle_cancel(const std::shared_ptr<GoalHandleSetAngle>)
        {
            RCLCPP_WARN(this->get_logger(), "Goal canceled");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // 승인 후 실행
        void angle_handle_accepted(const std::shared_ptr<GoalHandleSetAngle> goal_handle)
        {
            std::thread{std::bind(&Hand_control::execute, this, goal_handle)}.detach();
        }

        // 액션 실행
        void execute(const std::shared_ptr<GoalHandleSetAngle> goal_handle)
        {
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<SetAngle::Feedback>();
            auto result = std::make_shared<SetAngle::Result>();

            // 실제 모터 명령 보내는 send_set_command() 는 너가 넣으면 됨
            std::array<uint16_t, 6> target = {
                goal->angle0, goal->angle1, goal->angle2,
                goal->angle3, goal->angle4, goal->angle5
            };

            bool success = send_set_command(goal->hand_id, target, angleset, "set_angle");
            if (!success)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send set_angle command");
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "set_angle command sent, monitoring...");

            rclcpp::Rate rate(10);
            int loop_count = 0;
            const int max_loop = 100; // 10초

            while (rclcpp::ok() && loop_count++ < max_loop)
            {
                // 피드백
                for(int i=0; i<6; i++)
                    feedback->current_angles[i] = angle_current[i];
                goal_handle->publish_feedback(feedback);

                // 목표 도달 확인
                bool reached = true;
                for(int i=0; i<6; i++)
                {
                    if (std::abs(static_cast<int>(angle_current[i]) - static_cast<int>(target[i])) > 15)
                    {
                        reached = false;
                        break;
                    }
                }
                if (reached) {
                    RCLCPP_INFO(this->get_logger(), "Goal reached!");
                    result->success = true;
                    goal_handle->succeed(result);
                    return;
                }
                rate.sleep();
            }

            // timeout
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_WARN(this->get_logger(), "Goal timeout / aborted");
        }

        rclcpp::Publisher<inspire_hand_interface::msg::Posact>::SharedPtr get_posact;
        rclcpp::Publisher<inspire_hand_interface::msg::Angleact>::SharedPtr get_angleact;
        rclcpp::Publisher<inspire_hand_interface::msg::Forceact>::SharedPtr get_forceact;
        rclcpp::Publisher<inspire_hand_interface::msg::Current>::SharedPtr get_current;
        rclcpp::Publisher<inspire_hand_interface::msg::Error>::SharedPtr get_error;
        rclcpp::Publisher<inspire_hand_interface::msg::Temp>::SharedPtr get_temp;

        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr little_pub_tip_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr little_pub_nail_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr little_pub_pad_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr ring_pub_tip_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr ring_pub_nail_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr ring_pub_pad_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr middle_pub_tip_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr middle_pub_nail_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr middle_pub_pad_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr index_pub_tip_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr index_pub_nail_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr index_pub_pad_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr thumb_pub_tip_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr thumb_pub_nail_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr thumb_pub_middle_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr thumb_pub_pad_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_palm_;

        rclcpp::TimerBase::SharedPtr timer1_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::executors::MultiThreadedExecutor exec;
    // auto node = std::make_shared<Hand_control>();
    // exec.add_node(node);
    // exec.spin();
    auto node = std::make_shared<Hand_control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
