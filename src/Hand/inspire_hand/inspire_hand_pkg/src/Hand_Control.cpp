#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

#include "inspire_hand_interface/msg/posact.hpp"
#include "inspire_hand_interface/msg/angleact.hpp"
#include "inspire_hand_interface/msg/forceact.hpp"
#include "inspire_hand_interface/msg/current.hpp"
#include "inspire_hand_interface/msg/error.hpp"
#include "inspire_hand_interface/msg/temp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

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

#define getpos_act         0x05FE
#define getangle_act       0x060A
#define getforce_act       0x062E
#define getcurrent         0x063A
#define geterror           0x0646
#define gettemp            0x0652

serial::Serial ros_ser;
unsigned char pub_send_buffer[64] = {0};
unsigned char pub_recv_buffer[256] = {0};
unsigned char send_buffer[64] = {0};
unsigned char recv_buffer[256] = {0};
unsigned short all_data[36] = {0};
rclcpp::WallRate loop_rate(25.0);        //40ms
// rclcpp::WallRate loop_rate(33.3);
// rclcpp::WallRate loop_rate(50.0);
std::mutex serial_mutex;

using std::placeholders::_1;
using std::placeholders::_2;

class Hand_control : public rclcpp::Node
{
    public:
        Hand_control() : Node("getparam_publisher")
        {
            try
            {
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
                ros_ser.flush();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            catch (serial::IOException &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Serial exception: %s", e.what());
                rclcpp::shutdown();
                return;
            }

            callback_group_setangle = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_setpos = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_setspeed = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_setforce = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_getangleact = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_getangleset = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_getposact = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_getposset = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_getspeedset = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_getforceact = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_getforceset = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_getcurrentact = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_geterror = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_gettemp = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
                std::chrono::milliseconds(20),
                std::bind(&Hand_control::publish_message, this)
            );

            Setangle_Server = this->create_service<inspire_hand_interface::srv::Setangle>("Setangle",
                            std::bind(&Hand_control::setangle_callback,this,_1,_2),
                            rmw_qos_profile_services_default,
                            callback_group_setangle);
            Setpos_Server = this->create_service<inspire_hand_interface::srv::Setpos>("Setpos",
                            std::bind(&Hand_control::setpos_callback,this,_1,_2),
                            rmw_qos_profile_services_default,
                            callback_group_setpos);
            Setspeed_Server = this->create_service<inspire_hand_interface::srv::Setspeed>("Setspeed",
                            std::bind(&Hand_control::setspeed_callback,this,_1,_2),
                            rmw_qos_profile_services_default,
                            callback_group_setspeed);
            Setforce_Server = this->create_service<inspire_hand_interface::srv::Setforce>("Setforce",
                            std::bind(&Hand_control::setforce_callback,this,_1,_2),
                            rmw_qos_profile_services_default,
                            callback_group_setforce);
            Getangleact_Server = this->create_service<inspire_hand_interface::srv::Getangleact>("Getangleact",
                            std::bind(&Hand_control::getangleact_callback, this, _1, _2),
                            rmw_qos_profile_services_default,
                            callback_group_getangleact);
            Getangleset_Server = this->create_service<inspire_hand_interface::srv::Getangleset>("Getangleset",
                            std::bind(&Hand_control::getangleset_callback, this, _1, _2),
                            rmw_qos_profile_services_default,
                            callback_group_getangleset);
            Getposact_Server = this->create_service<inspire_hand_interface::srv::Getposact>("Getposact",
                            std::bind(&Hand_control::getposact_callback, this, _1, _2),
                            rmw_qos_profile_services_default,
                            callback_group_getposact);
            Getposset_Server = this->create_service<inspire_hand_interface::srv::Getposset>("Getposset",
                            std::bind(&Hand_control::getposset_callback, this, _1, _2),
                            rmw_qos_profile_services_default,
                            callback_group_getposset);
            Getspeedset_Server = this->create_service<inspire_hand_interface::srv::Getspeedset>("Getspeedset",
                            std::bind(&Hand_control::getspeedset_callback, this, _1, _2),
                            rmw_qos_profile_services_default,
                            callback_group_getspeedset);
            Getforceact_Server = this->create_service<inspire_hand_interface::srv::Getforceact>("Getforceact",
                            std::bind(&Hand_control::getforceact_callback, this, _1, _2),
                            rmw_qos_profile_services_default,
                            callback_group_getforceact);
            Getforceset_Server = this->create_service<inspire_hand_interface::srv::Getforceset>("Getforceset",
                            std::bind(&Hand_control::getforceset_callback, this, _1, _2),
                            rmw_qos_profile_services_default,
                            callback_group_getforceset);
            Getcurrentact_Server = this->create_service<inspire_hand_interface::srv::Getcurrentact>("Getcurrentact",
                            std::bind(&Hand_control::getcurrentact_callback, this, _1, _2),
                            rmw_qos_profile_services_default,
                            callback_group_getcurrentact);
            Geterror_Server = this->create_service<inspire_hand_interface::srv::Geterror>("Geterror",
                            std::bind(&Hand_control::geterror_callback, this, _1, _2),
                            rmw_qos_profile_services_default,
                            callback_group_geterror);
            Gettemp_Server = this->create_service<inspire_hand_interface::srv::Gettemp>("Gettemp",
                            std::bind(&Hand_control::gettemp_callback, this, _1, _2),
                            rmw_qos_profile_services_default,
                            callback_group_gettemp);
        }

    private:
        // 서비스 콜백 그룹 선언
        rclcpp::CallbackGroup::SharedPtr callback_group_setangle;
        rclcpp::CallbackGroup::SharedPtr callback_group_setpos;
        rclcpp::CallbackGroup::SharedPtr callback_group_setspeed;
        rclcpp::CallbackGroup::SharedPtr callback_group_setforce;
        rclcpp::CallbackGroup::SharedPtr callback_group_getangleact;
        rclcpp::CallbackGroup::SharedPtr callback_group_getangleset;
        rclcpp::CallbackGroup::SharedPtr callback_group_getposact;
        rclcpp::CallbackGroup::SharedPtr callback_group_getposset;
        rclcpp::CallbackGroup::SharedPtr callback_group_getspeedset;
        rclcpp::CallbackGroup::SharedPtr callback_group_getforceact;
        rclcpp::CallbackGroup::SharedPtr callback_group_getforceset;
        rclcpp::CallbackGroup::SharedPtr callback_group_getcurrentact;
        rclcpp::CallbackGroup::SharedPtr callback_group_geterror;
        rclcpp::CallbackGroup::SharedPtr callback_group_gettemp;
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

        // 1486~1497 : ANGLE_SET(W)
        // void setangle_callback(const inspire_hand_interface::srv::Setangle::Request::SharedPtr request,
        //     const inspire_hand_interface::srv::Setangle::Response::SharedPtr response)
        // {
        //     u_int8_t check_sum = 0;

        //     if(request->status == "set_angle")
        //     {
        //         RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
        //         RCLCPP_INFO(this->get_logger(), "1111111111111");
        //         std::lock_guard<std::mutex> lock(serial_mutex);

        //         send_buffer[0] = 0xEB;
        //         send_buffer[1] = 0x90;
        //         send_buffer[2] = request->hand_id;
        //         send_buffer[3] = 0x0F;
        //         send_buffer[4] = 0x12;
        //         send_buffer[5] = 0xCE;
        //         send_buffer[6] = 0x05;
        //         send_buffer[7] = (request->angle0 & 0xFF);
        //         send_buffer[8] = ((request->angle0 >> 8) & 0xFF);
        //         send_buffer[9] = (request->angle1 & 0xFF);
        //         send_buffer[10] = ((request->angle1 >> 8) & 0xFF);
        //         send_buffer[11] = (request->angle2 & 0xFF);
        //         send_buffer[12] = ((request->angle2 >> 8) & 0xFF);
        //         send_buffer[13] = (request->angle3 & 0xFF);
        //         send_buffer[14] = ((request->angle3 >> 8) & 0xFF);
        //         send_buffer[15] = (request->angle4 & 0xFF);
        //         send_buffer[16] = ((request->angle4 >> 8) & 0xFF);
        //         send_buffer[17] = (request->angle5 & 0xFF);
        //         send_buffer[18] = ((request->angle5 >> 8) & 0xFF);
        //         for(int i = 2;i < 19;i++)
        //         {
        //             check_sum += send_buffer[i];
        //         }
        //         send_buffer[19] = check_sum;
        //         ros_ser.write(send_buffer,20);
        //         loop_rate.sleep();
        //         int count = ros_ser.available();
        //         RCLCPP_INFO(this->get_logger(), "%d", count);
        //         if (count != 0)
        //         {
        //             std::vector<unsigned char> recv_buffer(count);//recv 버퍼
        //             count = ros_ser.read(&recv_buffer[0], count); //읽은 데이터 버퍼 수
        //             if(recv_buffer[7] == 0x01)
        //             {
        //                 response->angle_accepted = true;
        //                 printf("set_angle success\n");
        //             }
        //             else
        //             {
        //                 response->angle_accepted = false;
        //                 printf("set_angle fail\n");
        //             }
        //         }
        //     }
        //     else
        //     {
        //     //명령 오류
        //     response->angle_accepted = false;
        //     RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.:%s", request->status.c_str());
        //     }
        // }

        void setangle_callback(const inspire_hand_interface::srv::Setangle::Request::SharedPtr request,
            const inspire_hand_interface::srv::Setangle::Response::SharedPtr response)
        {
            if (request->status == "set_angle")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전 버퍼 플러시
                while (ros_ser.available())
                {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                // 패킷 생성
                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x0F;
                send_buffer[4] = 0x12;
                send_buffer[5] = 0xCE;
                send_buffer[6] = 0x05;
                send_buffer[7] = (request->angle0 & 0xFF);
                send_buffer[8] = ((request->angle0 >> 8) & 0xFF);
                send_buffer[9] = (request->angle1 & 0xFF);
                send_buffer[10] = ((request->angle1 >> 8) & 0xFF);
                send_buffer[11] = (request->angle2 & 0xFF);
                send_buffer[12] = ((request->angle2 >> 8) & 0xFF);
                send_buffer[13] = (request->angle3 & 0xFF);
                send_buffer[14] = ((request->angle3 >> 8) & 0xFF);
                send_buffer[15] = (request->angle4 & 0xFF);
                send_buffer[16] = ((request->angle4 >> 8) & 0xFF);
                send_buffer[17] = (request->angle5 & 0xFF);
                send_buffer[18] = ((request->angle5 >> 8) & 0xFF);

                for (int i = 2; i < 19; ++i)
                    check_sum += send_buffer[i];
                send_buffer[19] = check_sum;

                // 전송
                ros_ser.write(send_buffer, 20);
                ros_ser.flush();

                // 응답 대기
                size_t expected_size = 9;  // 예: 헤더+명령+상태+체크섬 (변형 가능)
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout waiting for set_angle response");
                        response->angle_accepted = false;
                        return;
                    }
                }

                // 응답 검증
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                    recv_buffer[1] == 0x90 && recv_buffer[2] == request->hand_id && recv_buffer[4] == 0x12)
                {
                    if (recv_buffer[7] == 0x01)
                    {
                        response->angle_accepted = true;
                        RCLCPP_INFO(this->get_logger(), "set_angle success");
                    }
                    else
                    {
                        response->angle_accepted = false;
                        RCLCPP_WARN(this->get_logger(), "set_angle failed, response code: 0x%02X", recv_buffer[7]);
                    }
                }
                else
                {
                    response->angle_accepted = false;
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response for set_angle");
                }
            }
            else
            {
                response->angle_accepted = false;
                RCLCPP_INFO(this->get_logger(), "status가 set_angle이 아님: %s", request->status.c_str());
            }
        }


        // 1474~1485 : POS_SET(W)
        // void setpos_callback(const inspire_hand_interface::srv::Setpos::Request::SharedPtr request,
        //     const inspire_hand_interface::srv::Setpos::Response::SharedPtr response)
        // {
        //     u_int8_t check_sum = 0;

        //     if(request->status == "set_pos")
        //     {
        //         RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
        //         std::lock_guard<std::mutex> lock(serial_mutex);

        //         send_buffer[0] = 0xEB;
        //         send_buffer[1] = 0x90;
        //         send_buffer[2] = request->hand_id;
        //         send_buffer[3] = 0x0F;
        //         send_buffer[4] = 0x12;
        //         send_buffer[5] = 0xC2;
        //         send_buffer[6] = 0x05;
        //         send_buffer[7] = (request->pos0 & 0xFF);
        //         send_buffer[8] = ((request->pos0 >> 8) & 0xFF);
        //         send_buffer[9] = (request->pos1 & 0xFF);
        //         send_buffer[10] = ((request->pos1 >> 8) & 0xFF);
        //         send_buffer[11] = (request->pos2 & 0xFF);
        //         send_buffer[12] = ((request->pos2 >> 8) & 0xFF);
        //         send_buffer[13] = (request->pos3 & 0xFF);
        //         send_buffer[14] = ((request->pos3 >> 8) & 0xFF);
        //         send_buffer[15] = (request->pos4 & 0xFF);
        //         send_buffer[16] = ((request->pos4 >> 8) & 0xFF);
        //         send_buffer[17] = (request->pos5 & 0xFF);
        //         send_buffer[18] = ((request->pos5 >> 8) & 0xFF);
        //         for(int i = 2;i < 19;i++)
        //         {
        //             check_sum += send_buffer[i];
        //         }
        //         send_buffer[19] = check_sum;
        //         ros_ser.write(send_buffer,20);
        //         loop_rate.sleep();    //100ms
        //         int count = ros_ser.available();
        //         if (count != 0)
        //         {
        //             std::vector<unsigned char> recv_buffer(count);
        //             count = ros_ser.read(&recv_buffer[0], count);
        //             if(recv_buffer[7] == 0x01)
        //             {
        //                 response->pos_accepted = true;
        //                 printf("set_pos success\n");
        //             }
        //             else
        //             {
        //                 response->pos_accepted = false;
        //                 printf("set_pos fail\n");
        //             }
        //         }
        //     }
        //     else
        //     {
        //         response->pos_accepted = false;
        //         RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.:%s", request->status.c_str());
        //     }
        // }

        void setpos_callback(const inspire_hand_interface::srv::Setpos::Request::SharedPtr request,
            const inspire_hand_interface::srv::Setpos::Response::SharedPtr response)
        {
            if (request->status == "set_pos")
            {
                RCLCPP_INFO(this->get_logger(), "수신된 status: %s", request->status.c_str());

                std::lock_guard<std::mutex> lock(serial_mutex);

                // 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                // 전송 패킷 구성
                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x0F;
                send_buffer[4] = 0x12;
                send_buffer[5] = 0xC2;
                send_buffer[6] = 0x05;
                send_buffer[7] = request->pos0 & 0xFF;
                send_buffer[8] = (request->pos0 >> 8) & 0xFF;
                send_buffer[9] = request->pos1 & 0xFF;
                send_buffer[10] = (request->pos1 >> 8) & 0xFF;
                send_buffer[11] = request->pos2 & 0xFF;
                send_buffer[12] = (request->pos2 >> 8) & 0xFF;
                send_buffer[13] = request->pos3 & 0xFF;
                send_buffer[14] = (request->pos3 >> 8) & 0xFF;
                send_buffer[15] = request->pos4 & 0xFF;
                send_buffer[16] = (request->pos4 >> 8) & 0xFF;
                send_buffer[17] = request->pos5 & 0xFF;
                send_buffer[18] = (request->pos5 >> 8) & 0xFF;

                for (int i = 2; i < 19; ++i)
                    check_sum += send_buffer[i];
                send_buffer[19] = check_sum;

                // 패킷 전송
                ros_ser.write(send_buffer, 20);
                ros_ser.flush();  // 중요

                // 응답 수신 대기
                size_t expected_size = 9;
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);  // 최소 예상 크기

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout while waiting for response from hand.");
                        response->pos_accepted = false;
                        return;
                    }
                }

                // 응답 검증
                if (recv_buffer.size() >= 8 &&
                    recv_buffer[0] == 0xEB && recv_buffer[1] == 0x90 && recv_buffer[4] == 0x12)
                {
                    if (recv_buffer[7] == 0x01)
                    {
                        response->pos_accepted = true;
                        RCLCPP_INFO(this->get_logger(), "set_pos success");
                    }
                    else
                    {
                        response->pos_accepted = false;
                        RCLCPP_WARN(this->get_logger(), "set_pos fail: 응답 바이트가 0x01이 아님 (0x%02X)", recv_buffer[7]);
                    }
                }
                else
                {
                    response->pos_accepted = false;
                    RCLCPP_WARN(this->get_logger(), "Invalid or malformed response received.");
                }
            }
        }


        // 1522~1533 : SPEED_SET(W)
        // void setspeed_callback(const inspire_hand_interface::srv::Setspeed::Request::SharedPtr request,
        // const inspire_hand_interface::srv::Setspeed::Response::SharedPtr response)
        // {
        //     u_int8_t check_sum = 0;

        //     if(request->status == "set_speed")
        //     {
        //         RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
        //         std::lock_guard<std::mutex> lock(serial_mutex);

        //         send_buffer[0] = 0xEB;
        //         send_buffer[1] = 0x90;
        //         send_buffer[2] = request->hand_id;
        //         send_buffer[3] = 0x0F;
        //         send_buffer[4] = 0x12;
        //         send_buffer[5] = 0xF2;
        //         send_buffer[6] = 0x05;
        //         send_buffer[7] = (request->speed0 & 0xFF);
        //         send_buffer[8] = ((request->speed0 >> 8) & 0xFF);
        //         send_buffer[9] = (request->speed1 & 0xFF);
        //         send_buffer[10] = ((request->speed1 >> 8) & 0xFF);
        //         send_buffer[11] = (request->speed2 & 0xFF);
        //         send_buffer[12] = ((request->speed2 >> 8) & 0xFF);
        //         send_buffer[13] = (request->speed3 & 0xFF);
        //         send_buffer[14] = ((request->speed3 >> 8) & 0xFF);
        //         send_buffer[15] = (request->speed4 & 0xFF);
        //         send_buffer[16] = ((request->speed4 >> 8) & 0xFF);
        //         send_buffer[17] = (request->speed5 & 0xFF);
        //         send_buffer[18] = ((request->speed5 >> 8) & 0xFF);
        //         for(int i = 2;i < 19;i++)
        //         {
        //             check_sum += send_buffer[i];
        //         }
        //         send_buffer[19] = check_sum;
        //         ros_ser.write(send_buffer,20);
        //         loop_rate.sleep();    //100ms
        //         int count = ros_ser.available();
        //         if (count != 0)
        //         {
        //             std::vector<unsigned char> recv_buffer(count);
        //             count = ros_ser.read(&recv_buffer[0], count);
        //             if(recv_buffer[7] == 0x01)
        //             {
        //                 response->speed_accepted = true;
        //                 printf("set_speed success\n");
        //             }
        //             else
        //             {
        //                 response->speed_accepted = false;
        //                 printf("set_speed fail\n");
        //             }
        //         }
        //     }
        //     else
        //     {
        //         response->speed_accepted = false;
        //         RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.:%s", request->status.c_str());
        //     }
        // }

        void setspeed_callback(const inspire_hand_interface::srv::Setspeed::Request::SharedPtr request,
            const inspire_hand_interface::srv::Setspeed::Response::SharedPtr response)
        {
            if (request->status == "set_speed")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전 버퍼 플러시
                while (ros_ser.available())
                {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                // 패킷 생성
                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x0F;
                send_buffer[4] = 0x12;
                send_buffer[5] = 0xF2;
                send_buffer[6] = 0x05;
                send_buffer[7] = (request->speed0 & 0xFF);
                send_buffer[8] = ((request->speed0 >> 8) & 0xFF);
                send_buffer[9] = (request->speed1 & 0xFF);
                send_buffer[10] = ((request->speed1 >> 8) & 0xFF);
                send_buffer[11] = (request->speed2 & 0xFF);
                send_buffer[12] = ((request->speed2 >> 8) & 0xFF);
                send_buffer[13] = (request->speed3 & 0xFF);
                send_buffer[14] = ((request->speed3 >> 8) & 0xFF);
                send_buffer[15] = (request->speed4 & 0xFF);
                send_buffer[16] = ((request->speed4 >> 8) & 0xFF);
                send_buffer[17] = (request->speed5 & 0xFF);
                send_buffer[18] = ((request->speed5 >> 8) & 0xFF);

                for (int i = 2; i < 19; ++i)
                    check_sum += send_buffer[i];
                send_buffer[19] = check_sum;

                // 전송
                ros_ser.write(send_buffer, 20);
                ros_ser.flush();

                // 응답 대기
                size_t expected_size = 9;  // EB 90 ID LEN CMD ... checksum
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout waiting for set_speed response");
                        response->speed_accepted = false;
                        return;
                    }
                }

                // 응답 검증
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                recv_buffer[1] == 0x90 && recv_buffer[2] == request->hand_id && recv_buffer[4] == 0x12)
                {
                    if (recv_buffer[7] == 0x01)
                    {
                        response->speed_accepted = true;
                        RCLCPP_INFO(this->get_logger(), "set_speed success");
                    }
                    else
                    {
                        response->speed_accepted = false;
                        RCLCPP_WARN(this->get_logger(), "set_speed failed, response code: 0x%02X", recv_buffer[7]);
                    }
                }
                else
                {
                    response->speed_accepted = false;
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response for set_speed");
                }
            }
            else
            {
                response->speed_accepted = false;
                RCLCPP_INFO(this->get_logger(), "status가 set_speed이 아님: %s", request->status.c_str());
            }
        }

        // 1498~1521 : FORCE_SET(W)
        // void setforce_callback(const inspire_hand_interface::srv::Setforce::Request::SharedPtr request,
        //     const inspire_hand_interface::srv::Setforce::Response::SharedPtr response)
        // {
        //     u_int8_t check_sum = 0;

        //     if(request->status == "set_force")
        //     {
        //         RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
        //         std::lock_guard<std::mutex> lock(serial_mutex);

        //         send_buffer[0] = 0xEB;
        //         send_buffer[1] = 0x90;
        //         send_buffer[2] = request->hand_id;
        //         send_buffer[3] = 0x0F;
        //         send_buffer[4] = 0x12;
        //         send_buffer[5] = 0xDA;
        //         send_buffer[6] = 0x05;
        //         send_buffer[7] = (request->force0 & 0xFF);
        //         send_buffer[8] = ((request->force0 >> 8) & 0xFF);
        //         send_buffer[9] = (request->force1 & 0xFF);
        //         send_buffer[10] = ((request->force1 >> 8) & 0xFF);
        //         send_buffer[11] = (request->force2 & 0xFF);
        //         send_buffer[12] = ((request->force2 >> 8) & 0xFF);
        //         send_buffer[13] = (request->force3 & 0xFF);
        //         send_buffer[14] = ((request->force3 >> 8) & 0xFF);
        //         send_buffer[15] = (request->force4 & 0xFF);
        //         send_buffer[16] = ((request->force4 >> 8) & 0xFF);
        //         send_buffer[17] = (request->force5 & 0xFF);
        //         send_buffer[18] = ((request->force5 >> 8) & 0xFF);
        //         for(int i = 2;i < 19;i++)
        //         {
        //             check_sum += send_buffer[i];
        //         }
        //         send_buffer[19] = check_sum;
        //         ros_ser.write(send_buffer,20);
        //         loop_rate.sleep();    //100ms
        //         int count = ros_ser.available();
        //         if (count != 0)
        //         {
        //             std::vector<unsigned char> recv_buffer(count);
        //             count = ros_ser.read(&recv_buffer[0], count);
        //             if(recv_buffer[7] == 0x01)
        //             {
        //                 response->force_accepted = true;
        //                 printf("set_force sucess\n");
        //             }
        //             else
        //             {
        //                 response->force_accepted = false;
        //                 printf("set_force fail\n");
        //             }
        //         }
        //     }
        //     else
        //     {
        //         response->force_accepted = false;
        //         RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.:%s", request->status.c_str());
        //     }
        // }

        void setforce_callback(const inspire_hand_interface::srv::Setforce::Request::SharedPtr request,
            const inspire_hand_interface::srv::Setforce::Response::SharedPtr response)
        {
            if (request->status == "set_force")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
                std::lock_guard<std::mutex> lock(serial_mutex);

                // 이전 응답 제거
                while (ros_ser.available())
                {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                // 패킷 생성
                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x0F;
                send_buffer[4] = 0x12;
                send_buffer[5] = 0xDA;
                send_buffer[6] = 0x05;
                send_buffer[7] = (request->force0 & 0xFF);
                send_buffer[8] = ((request->force0 >> 8) & 0xFF);
                send_buffer[9] = (request->force1 & 0xFF);
                send_buffer[10] = ((request->force1 >> 8) & 0xFF);
                send_buffer[11] = (request->force2 & 0xFF);
                send_buffer[12] = ((request->force2 >> 8) & 0xFF);
                send_buffer[13] = (request->force3 & 0xFF);
                send_buffer[14] = ((request->force3 >> 8) & 0xFF);
                send_buffer[15] = (request->force4 & 0xFF);
                send_buffer[16] = ((request->force4 >> 8) & 0xFF);
                send_buffer[17] = (request->force5 & 0xFF);
                send_buffer[18] = ((request->force5 >> 8) & 0xFF);

                for (int i = 2; i < 19; ++i)
                    check_sum += send_buffer[i];
                send_buffer[19] = check_sum;

                // 전송
                ros_ser.write(send_buffer, 20);
                ros_ser.flush();

                // 응답 대기
                size_t expected_size = 9;
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout waiting for set_force response");
                        response->force_accepted = false;
                        return;
                    }
                }

                // 응답 검증
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                    recv_buffer[1] == 0x90 && recv_buffer[2] == request->hand_id && recv_buffer[4] == 0x12)
                {
                    if (recv_buffer[7] == 0x01)
                    {
                        response->force_accepted = true;
                        RCLCPP_INFO(this->get_logger(), "set_force success");
                    }
                    else
                    {
                        response->force_accepted = false;
                        RCLCPP_WARN(this->get_logger(), "set_force failed, response code: 0x%02X", recv_buffer[7]);
                    }
                }
                else
                {
                    response->force_accepted = false;
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response for set_force");
                }
            }
            else
            {
                response->force_accepted = false;
                RCLCPP_INFO(this->get_logger(), "status가 set_force가 아님: %s", request->status.c_str());
            }
        }


        //1486~1497 : ANGLE_SET(R)
        void getangleset_callback(const inspire_hand_interface::srv::Getangleset::Request::SharedPtr request,
                const inspire_hand_interface::srv::Getangleset::Response::SharedPtr response)
        {
            if(request->status == "get_angleset")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전에 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = 0xCE;
                send_buffer[6] = 0x05;
                send_buffer[7] = 0x0C;
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();  // 중요

                // 응답 수신 대기
                size_t expected_size = 7 + 12 + 1;  // 6개 uint16 = 12바이트
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout reading serial response");
                        return;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                recv_buffer[1] == 0x90 && recv_buffer[4] == 0x11)
                {
                    try {
                        for (size_t i = 0; i < 6; ++i)
                        {
                            // 7 + i*2, 8 + i*2 인덱스에서 값을 읽음
                            uint16_t val = (recv_buffer[7 + i * 2] & 0xFF) | ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
                            response->curangleset[i] = static_cast<int32_t>(val);
                        }

                        RCLCPP_INFO(this->get_logger(), "각도 set : %d %d %d %d %d %d",
                                response->curangleset[0], response->curangleset[1], response->curangleset[2],
                                response->curangleset[3], response->curangleset[4], response->curangleset[5]);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Exception during response parsing: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response received.");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleact가 아님: %s", request->status.c_str());
            }
        }

        // 1474~1485 : POS_SET(R)
        void getposset_callback(const inspire_hand_interface::srv::Getposset::Request::SharedPtr request,
                const inspire_hand_interface::srv::Getposset::Response::SharedPtr response)
        {
            if(request->status == "get_posset")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전에 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = 0xC2;
                send_buffer[6] = 0x05;
                send_buffer[7] = 0x0C;
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();  // 중요

                // 응답 수신 대기
                size_t expected_size = 7 + 12 + 1;  // 6개 uint16 = 12바이트
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout reading serial response");
                        return;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                recv_buffer[1] == 0x90 && recv_buffer[4] == 0x11)
                {
                    try {
                        for (size_t i = 0; i < 6; ++i)
                        {
                            // 7 + i*2, 8 + i*2 인덱스에서 값을 읽음
                            uint16_t val = (recv_buffer[7 + i * 2] & 0xFF) | ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
                            response->curposset[i] = static_cast<int32_t>(val);
                        }

                        RCLCPP_INFO(this->get_logger(), "포즈 set : %d %d %d %d %d %d",
                                response->curposset[0], response->curposset[1], response->curposset[2],
                                response->curposset[3], response->curposset[4], response->curposset[5]);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Exception during response parsing: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response received.");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleact가 아님: %s", request->status.c_str());
            }
        }

        // 1522~1533 : SPEED_SET(R)
        void getspeedset_callback(const inspire_hand_interface::srv::Getspeedset::Request::SharedPtr request,
                const inspire_hand_interface::srv::Getspeedset::Response::SharedPtr response)
        {
            if(request->status == "get_speedset")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전에 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = 0xF2;
                send_buffer[6] = 0x05;
                send_buffer[7] = 0x0C;
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();  // 중요

                // 응답 수신 대기
                size_t expected_size = 7 + 12 + 1;  // 6개 uint16 = 12바이트
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout reading serial response");
                        return;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                recv_buffer[1] == 0x90 && recv_buffer[4] == 0x11)
                {
                    try {
                        for (size_t i = 0; i < 6; ++i)
                        {
                            // 7 + i*2, 8 + i*2 인덱스에서 값을 읽음
                            uint16_t val = (recv_buffer[7 + i * 2] & 0xFF) | ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
                            response->curspeedset[i] = static_cast<int32_t>(val);
                        }

                        RCLCPP_INFO(this->get_logger(), "스피드 set : %d %d %d %d %d %d",
                                response->curspeedset[0], response->curspeedset[1], response->curspeedset[2],
                                response->curspeedset[3], response->curspeedset[4], response->curspeedset[5]);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Exception during response parsing: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response received.");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleact가 아님: %s", request->status.c_str());
            }
        }

        // 1498~1521 : FORCE_SET(R)
        void getforceset_callback(const inspire_hand_interface::srv::Getforceset::Request::SharedPtr request,
                const inspire_hand_interface::srv::Getforceset::Response::SharedPtr response)
        {
            if(request->status == "get_forceset")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전에 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = 0xFE;
                send_buffer[6] = 0x05;
                send_buffer[7] = 0x0C;
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();  // 중요

                // 응답 수신 대기
                size_t expected_size = 7 + 12 + 1;  // 6개 uint16 = 12바이트
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout reading serial response");
                        return;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                recv_buffer[1] == 0x90 && recv_buffer[4] == 0x11)
                {
                    try {
                        for (size_t i = 0; i < 6; ++i)
                        {
                            // 7 + i*2, 8 + i*2 인덱스에서 값을 읽음
                            uint16_t val = (recv_buffer[7 + i * 2] & 0xFF) | ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
                            response->curforceset[i] = static_cast<int32_t>(val);
                        }

                        RCLCPP_INFO(this->get_logger(), "포스 set : %d %d %d %d %d %d",
                                response->curforceset[0], response->curforceset[1], response->curforceset[2],
                                response->curforceset[3], response->curforceset[4], response->curforceset[5]);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Exception during response parsing: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response received.");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleact가 아님: %s", request->status.c_str());
            }
        }

        //1546~1545 : ANGLE_ACT(R)
        void getangleact_callback(const inspire_hand_interface::srv::Getangleact::Request::SharedPtr request,
            const inspire_hand_interface::srv::Getangleact::Response::SharedPtr response)
        {
            if (request->status == "get_angleact")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전에 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = 0x0A;
                send_buffer[6] = 0x06;
                send_buffer[7] = 0x0C;
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();  // 중요

                // 응답 수신 대기
                size_t expected_size = 7 + 12 + 1;  // 6개 uint16 = 12바이트
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout reading serial response");
                        return;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                recv_buffer[1] == 0x90 && recv_buffer[4] == 0x11)
                {
                    try {
                        for (size_t i = 0; i < 6; ++i)
                        {
                            // 7 + i*2, 8 + i*2 인덱스에서 값을 읽음
                            uint16_t val = (recv_buffer[7 + i * 2] & 0xFF) | ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
                            response->curangleact[i] = static_cast<int32_t>(val);
                        }

                        RCLCPP_INFO(this->get_logger(), "각도 act : %d %d %d %d %d %d",
                                response->curangleact[0], response->curangleact[1], response->curangleact[2],
                                response->curangleact[3], response->curangleact[4], response->curangleact[5]);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Exception during response parsing: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response received.");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleact가 아님: %s", request->status.c_str());
            }
        }

        // 1534~1545 : POS_ACT(R)
        void getposact_callback(const inspire_hand_interface::srv::Getposact::Request::SharedPtr request,
                const inspire_hand_interface::srv::Getposact::Response::SharedPtr response)
        {
            if(request->status == "get_posact")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전에 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = 0xFE;
                send_buffer[6] = 0x05;
                send_buffer[7] = 0x0C;
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();  // 중요

                // 응답 수신 대기
                size_t expected_size = 7 + 12 + 1;  // 6개 uint16 = 12바이트
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout reading serial response");
                        return;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                recv_buffer[1] == 0x90 && recv_buffer[4] == 0x11)
                {
                    try {
                        for (size_t i = 0; i < 6; ++i)
                        {
                            // 7 + i*2, 8 + i*2 인덱스에서 값을 읽음
                            uint16_t val = (recv_buffer[7 + i * 2] & 0xFF) | ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
                            response->curposact[i] = static_cast<int32_t>(val);
                        }

                        RCLCPP_INFO(this->get_logger(), "포즈 act : %d %d %d %d %d %d",
                                response->curposact[0], response->curposact[1], response->curposact[2],
                                response->curposact[3], response->curposact[4], response->curposact[5]);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Exception during response parsing: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response received.");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleact가 아님: %s", request->status.c_str());
            }
        }

        // 1582~1593 : FORCE_ACT(R)
        void getforceact_callback(const inspire_hand_interface::srv::Getforceact::Request::SharedPtr request,
                const inspire_hand_interface::srv::Getforceact::Response::SharedPtr response)
        {
            if(request->status == "get_forceact")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전에 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = 0x2E;
                send_buffer[6] = 0x06;
                send_buffer[7] = 0x0C;
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();  // 중요

                // 응답 수신 대기
                size_t expected_size = 7 + 12 + 1;  // 6개 uint16 = 12바이트
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout reading serial response");
                        return;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                recv_buffer[1] == 0x90 && recv_buffer[4] == 0x11)
                {
                    try {
                        for (size_t i = 0; i < 6; ++i)
                        {
                            // 7 + i*2, 8 + i*2 인덱스에서 값을 읽음
                            uint16_t val = (recv_buffer[7 + i * 2] & 0xFF) | ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
                            response->curforceact[i] = static_cast<int32_t>(val);
                        }

                        RCLCPP_INFO(this->get_logger(), "포스 act : %d %d %d %d %d %d",
                                response->curforceact[0], response->curforceact[1], response->curforceact[2],
                                response->curforceact[3], response->curforceact[4], response->curforceact[5]);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Exception during response parsing: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response received.");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleact가 아님: %s", request->status.c_str());
            }
        }

        // 1594~1595 : CURRENT(R)
        void getcurrentact_callback(const inspire_hand_interface::srv::Getcurrentact::Request::SharedPtr request,
                    const inspire_hand_interface::srv::Getcurrentact::Response::SharedPtr response)
        {
            if(request->status == "get_currentact")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전에 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = 0x3A;
                send_buffer[6] = 0x06;
                send_buffer[7] = 0x0C;
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();  // 중요

                // 응답 수신 대기
                size_t expected_size = 7 + 12 + 1;  // 6개 uint16 = 12바이트
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout reading serial response");
                        return;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                recv_buffer[1] == 0x90 && recv_buffer[4] == 0x11)
                {
                    try {
                        for (size_t i = 0; i < 6; ++i)
                        {
                            // 7 + i*2, 8 + i*2 인덱스에서 값을 읽음
                            uint16_t val = (recv_buffer[7 + i * 2] & 0xFF) | ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
                            response->curcurrent[i] = static_cast<int32_t>(val);
                        }

                        RCLCPP_INFO(this->get_logger(), "전류 act : %d %d %d %d %d %d",
                                response->curcurrent[0], response->curcurrent[1], response->curcurrent[2],
                                response->curcurrent[3], response->curcurrent[4], response->curcurrent[5]);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Exception during response parsing: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response received.");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleact가 아님: %s", request->status.c_str());
            }
        }

        // 1606~1611 : ERROR(R)
        void geterror_callback(const inspire_hand_interface::srv::Geterror::Request::SharedPtr request,
                const inspire_hand_interface::srv::Geterror::Response::SharedPtr response)
        {
            if(request->status == "get_error")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전에 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = 0x46;
                send_buffer[6] = 0x06;
                send_buffer[7] = 0x0C;
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();  // 중요

                // 응답 수신 대기
                size_t expected_size = 7 + 12 + 1;  // 6개 uint16 = 12바이트
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout reading serial response");
                        return;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                recv_buffer[1] == 0x90 && recv_buffer[4] == 0x11)
                {
                    try {
                        for (size_t i = 0; i < 6; ++i)
                        {
                            // 7 + i*2, 8 + i*2 인덱스에서 값을 읽음
                            uint16_t val = (recv_buffer[7 + i * 2] & 0xFF) | ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
                            response->error[i] = static_cast<int32_t>(val);
                        }

                        RCLCPP_INFO(this->get_logger(), "에러 : %d %d %d %d %d %d",
                                response->error[0], response->error[1], response->error[2],
                                response->error[3], response->error[4], response->error[5]);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Exception during response parsing: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response received.");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleact가 아님: %s", request->status.c_str());
            }
        }

        // 1618~1623 : TEMP(R)
        void gettemp_callback(const inspire_hand_interface::srv::Gettemp::Request::SharedPtr request,
                const inspire_hand_interface::srv::Gettemp::Response::SharedPtr response)
        {
            if(request->status == "get_temp")
            {
                RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

                std::lock_guard<std::mutex> lock(serial_mutex);

                // 요청 전에 시리얼 버퍼 비우기
                while (ros_ser.available()) {
                    std::vector<unsigned char> flush_buf(ros_ser.available());
                    ros_ser.read(flush_buf.data(), flush_buf.size());
                }

                uint8_t check_sum = 0;
                send_buffer[0] = 0xEB;
                send_buffer[1] = 0x90;
                send_buffer[2] = request->hand_id;
                send_buffer[3] = 0x04;
                send_buffer[4] = 0x11;
                send_buffer[5] = 0x52;
                send_buffer[6] = 0x06;
                send_buffer[7] = 0x0C;
                for (int i = 2; i < 8; ++i)
                    check_sum += send_buffer[i];
                send_buffer[8] = check_sum;

                ros_ser.write(send_buffer, 9);
                ros_ser.flush();  // 중요

                // 응답 수신 대기
                size_t expected_size = 7 + 12 + 1;  // 6개 uint16 = 12바이트
                std::vector<unsigned char> recv_buffer;
                recv_buffer.reserve(expected_size);

                auto start = std::chrono::steady_clock::now();
                while (recv_buffer.size() < expected_size)
                {
                    if (ros_ser.available())
                    {
                        size_t bytes_to_read = ros_ser.available();
                        std::vector<unsigned char> temp_buffer(bytes_to_read);
                        size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
                        recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100)
                    {
                        RCLCPP_WARN(this->get_logger(), "Timeout reading serial response");
                        return;
                    }
                }

                // 응답 검증 및 파싱
                if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
                recv_buffer[1] == 0x90 && recv_buffer[4] == 0x11)
                {
                    try {
                        for (size_t i = 0; i < 6; ++i)
                        {
                            // 7 + i*2, 8 + i*2 인덱스에서 값을 읽음
                            uint16_t val = (recv_buffer[7 + i * 2] & 0xFF) | ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
                            response->temp[i] = static_cast<int32_t>(val);
                        }

                        RCLCPP_INFO(this->get_logger(), "온도 : %d %d %d %d %d %d",
                                response->temp[0], response->temp[1], response->temp[2],
                                response->temp[3], response->temp[4], response->temp[5]);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Exception during response parsing: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid or no response received.");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "status가 get_angleact가 아님: %s", request->status.c_str());
            }
        }

        // std::vector<uint16_t> read_registers(uint8_t hand_id, uint16_t reg, uint8_t len)
        // {
        //     std::lock_guard<std::mutex> lock(serial_mutex);

        //     // 요청 전에 시리얼 버퍼 비우기
        //     while (ros_ser.available()) {
        //         size_t avail = ros_ser.available();
        //         std::vector<unsigned char> flush_buf(avail);
        //         size_t read_len = ros_ser.read(flush_buf.data(), avail);
        //         if (read_len == 0) break; // 읽은 데이터 없으면 탈출
        //     }

        //     // 1. 명령어 생성
        //     uint8_t checksum = 0;
        //     pub_send_buffer[0] = 0xEB;
        //     pub_send_buffer[1] = 0x90;
        //     pub_send_buffer[2] = hand_id;
        //     pub_send_buffer[3] = 0x04;
        //     pub_send_buffer[4] = 0x11;
        //     pub_send_buffer[5] = reg & 0xFF;
        //     pub_send_buffer[6] = reg >> 8;
        //     pub_send_buffer[7] = len;
        //     for (int i = 2; i < 8; i++) checksum += pub_send_buffer[i];
        //     pub_send_buffer[8] = checksum;

        //     ros_ser.write(pub_send_buffer, 9);
        //     ros_ser.flush();

        //     // 2. 응답 수신
        //     size_t expected_size = 7 + len * 2 + 1;  // +1 for checksum
        //     std::vector<unsigned char> recv_buffer;
        //     recv_buffer.reserve(expected_size);

        //     auto start = std::chrono::steady_clock::now();
        //     while (recv_buffer.size() < expected_size)
        //     {
        //         if (ros_ser.available())
        //         {
        //             size_t available_bytes = ros_ser.available();
        //             RCLCPP_DEBUG(this->get_logger(), "Available bytes: %zu", available_bytes);
        //             size_t bytes_to_read = ros_ser.available();
        //             std::vector<unsigned char> temp_buffer(bytes_to_read);
        //             size_t read_len = ros_ser.read(temp_buffer.data(), bytes_to_read);
        //             recv_buffer.insert(recv_buffer.end(), temp_buffer.begin(), temp_buffer.begin() + read_len);
        //         }

        //         auto now = std::chrono::steady_clock::now();
        //         if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 50)
        //         {
        //             RCLCPP_WARN(this->get_logger(), "Timeout reading serial response");
        //             return {};
        //         }
        //     }

        //     // 응답 검증 전에 recv_buffer 안에서 0xEB 위치 찾기
        //     auto it = std::find(recv_buffer.begin(), recv_buffer.end(), 0xEB);
        //     if (it == recv_buffer.end()) {
        //         RCLCPP_WARN(this->get_logger(), "No start byte 0xEB found in response");
        //         return {};
        //     }

        //     // 0xEB 위치까지 앞 데이터는 버리고 재배열
        //     size_t start_index = std::distance(recv_buffer.begin(), it);
        //     if (start_index > 0) {
        //         RCLCPP_WARN(this->get_logger(), "Discarding %zu bytes before start byte 0xEB", start_index);
        //         recv_buffer.erase(recv_buffer.begin(), it);
        //         for(int j = 0; j<len+8; j++)
        //         {
        //             RCLCPP_INFO(this->get_logger(), "RX[%d] : %x ", j, recv_buffer[j]);
        //         }
        //     }

        //     // 응답 검증 및 파싱
        //     if (recv_buffer.size() >= expected_size && recv_buffer[0] == 0xEB &&
        //         recv_buffer[1] == 0x90 && recv_buffer[4] == 0x11)
        //     {
        //         std::vector<uint16_t> data;
        //         try {
        //             for (size_t i = 0; i < len; ++i)
        //             {
        //                 size_t low_index = 7 + i * 2;
        //                 size_t high_index = 8 + i * 2;

        //                 if (high_index >= recv_buffer.size())
        //                 {
        //                     RCLCPP_WARN(this->get_logger(),
        //                                 "recv_buffer too short while parsing data (i=%zu, required_index=%zu), size=%zu",
        //                                 i, high_index, recv_buffer.size());
        //                     break;
        //                 }

        //                 data.push_back(recv_buffer[low_index] | (recv_buffer[high_index] << 8));
        //             }
        //         }
        //         catch (const std::exception &e)
        //         {
        //             RCLCPP_ERROR(this->get_logger(), "Exception during response parsing: %s", e.what());
        //             return {};
        //         }

        //         return data;
        //     }
        //     else
        //     {
        //         RCLCPP_WARN(this->get_logger(), "Invalid response format or header mismatch");
        //         // for(int j = 0; j<len+8; j++)
        //         // {
        //         //     RCLCPP_INFO(this->get_logger(), "RX : %x ", recv_buffer[j]);
        //         // }
        //         return {};
        //     }
        // }

        std::vector<uint16_t> parse_packet(const std::vector<unsigned char>& buffer, uint8_t len)
        {
            std::vector<uint16_t> data;

            for (size_t offset = 0; offset + 8 <= buffer.size(); ++offset)
            {
                if (buffer[offset] == 0xEB && buffer[offset + 1] == 0x90 && buffer[offset + 4] == 0x11)
                {
                    size_t total_len = 7 + len * 2 + 1;

                    if (offset + total_len > buffer.size())
                    {
                        // 패킷이 부족함
                        continue;
                    }

                    for (size_t i = 0; i < len; ++i)
                    {
                        size_t low_index = offset + 7 + i * 2;
                        size_t high_index = offset + 8 + i * 2;
                        uint16_t value = buffer[low_index] | (buffer[high_index] << 8);
                        data.push_back(value);
                    }

                    return data;  // 유효한 패킷 파싱 완료
                }
            }

            return {};  // 유효한 패킷 없음
        }

        std::vector<uint16_t> read_registers(uint8_t hand_id, uint16_t reg, uint8_t len)
        {
            std::lock_guard<std::mutex> lock(serial_mutex);

            while (ros_ser.available()) {
                std::vector<unsigned char> flush_buf(ros_ser.available());
                ros_ser.read(flush_buf.data(), flush_buf.size());
            }

            uint8_t checksum = 0;
            pub_send_buffer[0] = 0xEB;
            pub_send_buffer[1] = 0x90;
            pub_send_buffer[2] = hand_id;
            pub_send_buffer[3] = 0x04;
            pub_send_buffer[4] = 0x11;
            pub_send_buffer[5] = reg & 0xFF;
            pub_send_buffer[6] = reg >> 8;
            pub_send_buffer[7] = len;
            for (int i = 2; i < 8; i++) checksum += pub_send_buffer[i];
            pub_send_buffer[8] = checksum;

            ros_ser.write(pub_send_buffer, 9);
            ros_ser.flush();

            size_t expected_size = 7 + len * 2 + 1;
            std::vector<unsigned char> recv_buffer;
            recv_buffer.reserve(expected_size + 8);  // 여유를 둠

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
                        return parsed;
                }
            }

            RCLCPP_WARN(this->get_logger(), "Timeout or invalid packet during read_registers");
            return {};
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

            // auto data = send_hand_packet(1, getpos_act);
            auto data = read_registers(1, getpos_act, 36);
            for(int i=0; i<36; i++)
            {
                all_data[i] = data[i];
                // RCLCPP_INFO(this->get_logger(), "all_data[%d]: %d", i, all_data[i]);
            }

            // auto little_pad_msgtip_data = send_tip_packet(1, 0x0BB8);
            // auto little_pad_msgnail_data = send_nail_packet(1, 0x0BCA);
            // auto little_pad_msgpad_data = send_pad_packet(1, 0x0C8A);
            auto little_pad_msgtip_data = read_registers(1, 0x0BB8, 9);
            auto little_pad_msgnail_data = read_registers(1, 0x0BCA, 96);
            auto little_pad_msgpad_data = read_registers(1, 0x0C8A, 40);
            little_tip_msg.data.assign(little_pad_msgtip_data.begin(), little_pad_msgtip_data.end());
            little_nail_msg.data.assign(little_pad_msgnail_data.begin(), little_pad_msgnail_data.end());
            little_pad_msg.data.assign(little_pad_msgpad_data.begin(), little_pad_msgpad_data.end());

            // auto ring_tip_data = send_tip_packet(1, 0x0D2A);
            // auto ring_nail_data = send_nail_packet(1, 0x0D3C);
            // auto ring_pad_data = send_pad_packet(1, 0x0DFC);
            auto ring_tip_data = read_registers(1, 0x0D2A, 9);
            auto ring_nail_data = read_registers(1, 0x0D3C, 96);
            auto ring_pad_data = read_registers(1, 0x0DFC, 40);
            ring_tip_msg.data.assign(ring_tip_data.begin(), ring_tip_data.end());
            ring_nail_msg.data.assign(ring_nail_data.begin(), ring_nail_data.end());
            ring_pad_msg.data.assign(ring_pad_data.begin(), ring_pad_data.end());

            // auto middle_tip_data = send_tip_packet(1, 0x0E9C);
            // auto middle_nail_data = send_nail_packet(1, 0x0EAE);
            // auto middle_pad_data = send_pad_packet(1, 0x0F6E);
            auto middle_tip_data = read_registers(1, 0x0E9C, 9);
            auto middle_nail_data = read_registers(1, 0x0EAE, 96);
            auto middle_pad_data = read_registers(1, 0x0F6E, 40);
            middle_tip_msg.data.assign(middle_tip_data.begin(), middle_tip_data.end());
            middle_nail_msg.data.assign(middle_nail_data.begin(), middle_nail_data.end());
            middle_pad_msg.data.assign(middle_pad_data.begin(), middle_pad_data.end());

            // auto index_tip_data  = send_tip_packet(1, 0x100E);
            // auto index_nail_data = send_nail_packet(1, 0x1020);
            // auto index_pad_data  = send_pad_packet(1, 0x10E0);
            auto index_tip_data  = read_registers(1, 0x100E, 9);
            auto index_nail_data = read_registers(1, 0x1020, 96);
            auto index_pad_data  = read_registers(1, 0x10E0, 40);
            index_tip_msg.data.assign(index_tip_data.begin(), index_tip_data.end());
            index_nail_msg.data.assign(index_nail_data.begin(), index_nail_data.end());
            index_pad_msg.data.assign(index_pad_data.begin(), index_pad_data.end());

            // auto thumb_tip_data = send_tip_packet(1, 0x1180);
            // auto thumb_nail_data = send_nail_packet(1, 0x1192);
            // auto thumb_middle_data = send_tip_packet(1, 0x1152);
            // auto thumb_pad_data = send_nail_packet(1, 0x1264);
            auto thumb_tip_data = read_registers(1, 0x1180, 9);
            auto thumb_nail_data = read_registers(1, 0x1192, 96);
            auto thumb_middle_data = read_registers(1, 0x1152, 9);
            auto thumb_pad_data = read_registers(1, 0x1264, 96);
            thumb_tip_msg.data.assign(thumb_tip_data.begin(), thumb_tip_data.end());
            thumb_nail_msg.data.assign(thumb_nail_data.begin(), thumb_nail_data.end());
            thumb_middle_msg.data.assign(thumb_middle_data.begin(), thumb_middle_data.end());
            thumb_pad_msg.data.assign(thumb_pad_data.begin(), thumb_pad_data.end());

            // auto palm_data = send_palm_packet(1, 0x1324);
            auto palm_data = read_registers(1, 0x1324, 112);
            palm_msg.data.assign(palm_data.begin(), palm_data.end());

            // auto gpos_act = send_hand_packet(1, getpos_act);
            for (int i = 0; i < 6; i++)
                posact_msg.pos[i] = all_data[i];
            get_posact->publish(posact_msg);

            // auto gangle_act = send_hand_packet(1, getangle_act);
            for (int i = 6; i < 12; i++)
                angleact_msg.angle[i-6] = all_data[i];
            get_angleact->publish(angleact_msg);

            // auto gforce_act = send_hand_packet(1, getforce_act);
            for (int i = 12; i < 18; i++)
                forceact_msg.force[i-12] = all_data[i];
            get_forceact->publish(forceact_msg);

            // auto gcurrent = send_hand_packet(1, getcurrent);
            for (int i = 18; i < 24; i++)
                current_msg.current[i-18] = all_data[i];
            get_current->publish(current_msg);

            // auto gerror = send_hand_packet(1, geterror);
            for (int i = 24; i < 30; i++)
                error_msg.error[i-24] = all_data[i];
            get_error->publish(error_msg);

            // auto gtemp = send_hand_packet(1, gettemp);
            for (int i = 30; i < 36; i++)
                temp_msg.temp[i-30] = all_data[i];
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
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<Hand_control>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}



//-----------------------------------------------------------------------------
// #include <chrono>
// #include <memory>
// #include <thread>
// #include <mutex>

// #include "rclcpp/rclcpp.hpp"
// #include "serial/serial.h"
// #include "inspire_hand_interface/msg/posact.hpp"
// #include "inspire_hand_interface/msg/angleact.hpp"
// #include "inspire_hand_interface/msg/forceact.hpp"
// #include "inspire_hand_interface/msg/current.hpp"
// #include "inspire_hand_interface/msg/error.hpp"
// #include "inspire_hand_interface/msg/temp.hpp"
// #include "std_srvs/srv/set_bool.hpp"

// #define getpos_act         0x05FE
// #define getangle_act       0x060A
// #define getforce_act       0x062E
// #define getcurrent         0x063A
// #define geterror           0x0646
// #define gettemp            0x0652

// serial::Serial ros_ser;
// unsigned char send_buffer[64] = {0};
// unsigned char recv_buffer[64] = {0};
// rclcpp::WallRate loop_rate(100.0);

// std::mutex serial_mutex;

// class ParamPublisher : public rclcpp::Node
// {
// public:
//     ParamPublisher() : Node("getparam_publisher")
//     {
//         try
//         {
//             ros_ser.setPort("/dev/ttyUSB0");
//             ros_ser.setBaudrate(115200);
//             serial::Timeout to = serial::Timeout::simpleTimeout(1000);
//             ros_ser.setTimeout(to);
//             ros_ser.open();
//             if (ros_ser.isOpen()) {
//                 RCLCPP_INFO(this->get_logger(), "Serial port opened.");
//               } else {
//                 RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
//                 rclcpp::shutdown();
//                 return;
//               }
//               ros_ser.flush();
//               std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         }
//         catch (serial::IOException &e)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Serial exception: %s", e.what());
//             rclcpp::shutdown();
//             return;
//         }

//         // 토픽 생성
//         get_posact = this->create_publisher<inspire_hand_interface::msg::Posact>("hand_posact", 10);
//         get_angleact = this->create_publisher<inspire_hand_interface::msg::Angleact>("hand_angleact", 10);
//         get_forceact = this->create_publisher<inspire_hand_interface::msg::Forceact>("hand_forceact", 10);
//         get_current = this->create_publisher<inspire_hand_interface::msg::Current>("hand_current", 10);
//         get_error = this->create_publisher<inspire_hand_interface::msg::Error>("hand_error", 10);
//         get_temp = this->create_publisher<inspire_hand_interface::msg::Temp>("hand_temp", 10);

//         // 서비스 생성
//         service_ = this->create_service<std_srvs::srv::SetBool>(
//             "/publish_hand_data",
//             std::bind(&ParamPublisher::handle_service_request, this, std::placeholders::_1, std::placeholders::_2)
//         );

//         service_1 = this->create_service<std_srvs::srv::SetBool>(
//             "/publish_hand_data1",
//             std::bind(&ParamPublisher::handle_service_request1, this, std::placeholders::_1, std::placeholders::_2)
//         );

//         RCLCPP_INFO(this->get_logger(), "Service '/publish_hand_data' ready to handle requests.");

//         // timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&ParamPublisher::publish_message, this));
//         // timer_posact_ = this->create_wall_timer(
//         //     std::chrono::milliseconds(60),
//         //     std::bind(&ParamPublisher::publish_posact_message, this)
//         // );
//         // timer_angleact_ = this->create_wall_timer(
//         //     std::chrono::milliseconds(60),
//         //     std::bind(&ParamPublisher::publish_angleact_message, this)
//         // );
//         // timer_forceact_ = this->create_wall_timer(
//         //     std::chrono::milliseconds(60),
//         //     std::bind(&ParamPublisher::publish_forceact_message, this)
//         // );
//         // timer_current_ = this->create_wall_timer(
//         //     std::chrono::milliseconds(60),
//         //     std::bind(&ParamPublisher::publish_current_message, this)
//         // );
//         // timer_error_ = this->create_wall_timer(
//         //     std::chrono::milliseconds(60),
//         //     std::bind(&ParamPublisher::publish_error_message, this)
//         // );
//         // timer_temp_ = this->create_wall_timer(
//         //     std::chrono::milliseconds(60),
//         //     std::bind(&ParamPublisher::publish_temp_message, this)
//         // );

//         timer_posact_ = this->create_wall_timer(
//             std::chrono::milliseconds(60),
//             std::bind(&ParamPublisher::publish_message, this)
//         );
//     }

// private:

//     bool is_active_ = false;
//     bool is_active_1 = false;

//     // 서비스 요청 처리 함수
//     void handle_service_request(
//         const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
//         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
//     {
//         if (request->data)
//         {
//             RCLCPP_INFO(this->get_logger(), "SetBool request TRUE received. Publishing data...");
//             // publish_message();
//             response->success = true;
//             response->message = "Data published.";
//         }
//         else
//         {
//             RCLCPP_INFO(this->get_logger(), "SetBool request FALSE received. No data published.");
//             response->success = true;
//             response->message = "Request acknowledged. No data published.";
//         }
//         is_active_ = request->data;
//     }

//     void handle_service_request1(
//         const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
//         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
//     {
//         if (request->data)
//         {
//             RCLCPP_INFO(this->get_logger(), "SetBool request TRUE received. Publishing data...");
//             // publish_message();
//             response->success = true;
//             response->message = "Data published.";
//         }
//         else
//         {
//             RCLCPP_INFO(this->get_logger(), "SetBool request FALSE received. No data published.");
//             response->success = true;
//             response->message = "Request acknowledged. No data published.";
//         }
//         is_active_1 = request->data;
//     }

//     std::array<u_int16_t, 6> send_hand_packet(u_int8_t hand_id, u_int16_t reg)
//     {
//         std::array<u_int16_t, 6> data16 = {0};
//         u_int8_t check_sum = 0;
//         std::lock_guard<std::mutex> lock(serial_mutex);

//         send_buffer[0] = 0xEB;
//         send_buffer[1] = 0x90;
//         send_buffer[2] = hand_id;
//         send_buffer[3] = 0x04;
//         send_buffer[4] = 0x11;          // 11:read, 12:write
//         send_buffer[5] = reg & 0xFF;    // start addr L
//         send_buffer[6] = reg >> 8;      // start addr H
//         send_buffer[7] = 0x0C;          // length
//         for (int i = 2; i < 8; i++)
//         {
//             check_sum += send_buffer[i];
//         }
//         send_buffer[8] = check_sum;
//         ros_ser.write(send_buffer, 9);
//         loop_rate.sleep();  // wait 100ms for recv data
//         int count = ros_ser.available(); // check if data is available
//         RCLCPP_INFO(this->get_logger(), "count: %d", count);
//         if (count != 0)
//         {
//             std::vector<unsigned char> recv_buffer(count);
//             count = ros_ser.read(&recv_buffer[0], count);

//             // for(int j=0; j<count;j++)
//             // {
//             //     RCLCPP_INFO(this->get_logger(), "%d. data: %x", j, recv_buffer[j]);
//             // }

//             if ((recv_buffer[4] == 0x11) && (recv_buffer[0] == 0xEB && recv_buffer[1] == 0x90))
//             {
//                 for (int i = 0; i < 6; i++)
//                 {
//                     data16[i] = (recv_buffer[7 + i * 2] & 0xFF) + ((recv_buffer[8 + i * 2] << 8) & 0xFF00);
//                 }
//                 // RCLCPP_INFO(this->get_logger(), "Current pos: %d %d %d %d %d %d", data16[0], data16[1], data16[2], data16[3], data16[4], data16[5]);
//             }
//             else
//             {
//                 RCLCPP_WARN(this->get_logger(), "Unable to read values.");
//             }
//         }
//         return data16;
//     }

//     void publish_posact_message()
//     {
//         if (is_active_) {
//             // std::lock_guard<std::mutex> lock(serial_mutex);
//             auto posact_msg = inspire_hand_interface::msg::Posact();
//             auto gpos_act = send_hand_packet(1, getpos_act);
//             for (int i = 0; i < 6; i++) {
//                 posact_msg.pos[i] = gpos_act[i];
//             }
//             get_posact->publish(posact_msg);
//         }
//     }

//     void publish_angleact_message()
//     {
//         if (is_active_) {
//             // std::lock_guard<std::mutex> lock(serial_mutex);
//             auto angleact_msg = inspire_hand_interface::msg::Angleact();
//             auto gangle_act = send_hand_packet(1, getangle_act);
//             for (int i = 0; i < 6; i++) {
//                 angleact_msg.angle[i] = gangle_act[i];
//             }
//             get_angleact->publish(angleact_msg);
//         }
//     }

//     void publish_forceact_message()
//     {
//         if (is_active_) {
//             // std::lock_guard<std::mutex> lock(serial_mutex);
//             auto forceact_msg = inspire_hand_interface::msg::Forceact();
//             auto gforce_act = send_hand_packet(1, getforce_act);
//             for (int i = 0; i < 6; i++) {
//                 forceact_msg.force[i] = gforce_act[i];
//             }
//             get_forceact->publish(forceact_msg);
//         }
//     }

//     void publish_current_message()
//     {
//         if (is_active_) {
//             // std::lock_guard<std::mutex> lock(serial_mutex);
//             auto current_msg = inspire_hand_interface::msg::Current();
//             auto gcurrent = send_hand_packet(1, getcurrent);
//             for (int i = 0; i < 6; i++) {
//                 current_msg.current[i] = gcurrent[i];
//             }
//             get_current->publish(current_msg);
//         }
//     }

//     void publish_error_message()
//     {
//         if (is_active_) {
//             // std::lock_guard<std::mutex> lock(serial_mutex);
//             auto error_msg = inspire_hand_interface::msg::Error();
//             auto gerror = send_hand_packet(1, geterror);
//             for (int i = 0; i < 6; i++) {
//                 error_msg.error[i] = gerror[i];
//             }
//             get_error->publish(error_msg);
//         }
//     }

//     void publish_temp_message()
//     {
//         if (is_active_) {
//             // std::lock_guard<std::mutex> lock(serial_mutex);
//             auto temp_msg = inspire_hand_interface::msg::Temp();
//             auto gtemp = send_hand_packet(1, gettemp);
//             for (int i = 0; i < 6; i++) {
//                 temp_msg.temp[i] = gtemp[i];
//             }
//             get_temp->publish(temp_msg);
//         }
//     }

//     void publish_message()
//     {
//         if (is_active_)
//         {
//             auto posact_msg = inspire_hand_interface::msg::Posact();
//             auto angleact_msg = inspire_hand_interface::msg::Angleact();
//             auto forceact_msg = inspire_hand_interface::msg::Forceact();
//             auto current_msg = inspire_hand_interface::msg::Current();
//             auto error_msg = inspire_hand_interface::msg::Error();
//             auto temp_msg = inspire_hand_interface::msg::Temp();

//             auto gpos_act = send_hand_packet(1, getpos_act);
//             for (int i = 0; i < 6; i++)
//                 posact_msg.pos[i] = gpos_act[i];
//             get_posact->publish(posact_msg);

//             auto gangle_act = send_hand_packet(1, getangle_act);
//             for (int i = 0; i < 6; i++)
//                 angleact_msg.angle[i] = gangle_act[i];
//             get_angleact->publish(angleact_msg);

//             auto gforce_act = send_hand_packet(1, getforce_act);
//             for (int i = 0; i < 6; i++)
//                 forceact_msg.force[i] = gforce_act[i];
//             get_forceact->publish(forceact_msg);

//             auto gcurrent = send_hand_packet(1, getcurrent);
//             for (int i = 0; i < 6; i++)
//                 current_msg.current[i] = gcurrent[i];
//             get_current->publish(current_msg);

//             auto gerror = send_hand_packet(1, geterror);
//             for (int i = 0; i < 6; i++)
//                 error_msg.error[i] = gerror[i];
//             get_error->publish(error_msg);

//             auto gtemp = send_hand_packet(1, gettemp);
//             for (int i = 0; i < 6; i++)
//                 temp_msg.temp[i] = gtemp[i];
//             get_temp->publish(temp_msg);
//         }
//         else return;
//     }

//     rclcpp::Publisher<inspire_hand_interface::msg::Posact>::SharedPtr get_posact;
//     rclcpp::Publisher<inspire_hand_interface::msg::Angleact>::SharedPtr get_angleact;
//     rclcpp::Publisher<inspire_hand_interface::msg::Forceact>::SharedPtr get_forceact;
//     rclcpp::Publisher<inspire_hand_interface::msg::Current>::SharedPtr get_current;
//     rclcpp::Publisher<inspire_hand_interface::msg::Error>::SharedPtr get_error;
//     rclcpp::Publisher<inspire_hand_interface::msg::Temp>::SharedPtr get_temp;

//     rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
//     rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_1;

//     rclcpp::TimerBase::SharedPtr timer_posact_;
//     rclcpp::TimerBase::SharedPtr timer_angleact_;
//     rclcpp::TimerBase::SharedPtr timer_forceact_;
//     rclcpp::TimerBase::SharedPtr timer_current_;
//     rclcpp::TimerBase::SharedPtr timer_error_;
//     rclcpp::TimerBase::SharedPtr timer_temp_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::executors::MultiThreadedExecutor exec;
//     auto node = std::make_shared<ParamPublisher>();
//     exec.add_node(node);
//     exec.spin();
//     rclcpp::shutdown();
//     return 0;
// }
