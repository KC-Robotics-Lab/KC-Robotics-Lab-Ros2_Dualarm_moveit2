#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "inspire_hand_interface/msg/posact.hpp"
#include "inspire_hand_interface/msg/angleact.hpp"
#include "inspire_hand_interface/msg/forceact.hpp"
#include "inspire_hand_interface/msg/current.hpp"
#include "inspire_hand_interface/msg/error.hpp"
#include "inspire_hand_interface/msg/temp.hpp"


#define getpos_act         0x05FE
#define getangle_act       0x060A
#define getforce_act       0x062E
#define getcurrent         0x063A
#define geterror           0x0646
#define gettemp            0x0652

serial::Serial ros_ser;
unsigned char send_buffer[64] = {0};
unsigned char recv_buffer[64] = {0};
rclcpp::WallRate loop_rate(100.0);

class ParamPublisher : public rclcpp::Node
{
    public:
        ParamPublisher() : Node("getparam_publisher")
        {
            // 토픽 생성
            get_posact = this->create_publisher<inspire_hand_interface::msg::Posact>("hand_posact", 10);
            get_angleact = this->create_publisher<inspire_hand_interface::msg::Angleact>("hand_angleact", 10);
            get_forceact = this->create_publisher<inspire_hand_interface::msg::Forceact>("hand_forceact", 10);
            get_current = this->create_publisher<inspire_hand_interface::msg::Current>("hand_current", 10);
            get_error = this->create_publisher<inspire_hand_interface::msg::Error>("hand_error", 10);
            get_temp = this->create_publisher<inspire_hand_interface::msg::Temp>("hand_temp", 10);
            // timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PosPublisher::publish_message, this));
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ParamPublisher::publish_message, this));
        }

    private:

    std::array<u_int32_t, 6> send_hand_packet(u_int8_t hand_id, u_int16_t reg)
    {
        std::array<u_int32_t, 6> data32 = {0};
        u_int8_t check_sum = 0;

        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = hand_id;
        send_buffer[3] = 0x04;
        send_buffer[4] = 0x11;          // 11:read, 12:write
        send_buffer[5] = reg & 0xFF;    // start addr L
        send_buffer[6] = reg >> 8;      // start addr H
        send_buffer[7] = 0x0C;          // length
        for(int i = 2;i < 8;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[8] = check_sum;
        ros_ser.write(send_buffer,9);
        loop_rate.sleep();    //wait 100ms for recv data
        int count = ros_ser.available(); // data가 있는지 확인
        if (count != 0)
        {
            std::vector<unsigned char> recv_buffer(count);
            count = ros_ser.read(&recv_buffer[0], count);
            if(recv_buffer[4] == 0x11)
            {
                for(int i=0;i<6;i++)
                {
                    data32[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                }
                RCLCPP_INFO(this->get_logger(), "현재 pos : %d %d %d %d %d %d", data32[0],data32[1],data32[2],data32[3],data32[4],data32[5]);
            }
            else
            {
                printf("값을 읽을 수 없습니다.\n");
            }
        }
        return data32;
    }

    void publish_message()
    {
        auto posact_msg = inspire_hand_interface::msg::Posact();
        auto angleact_msg = inspire_hand_interface::msg::Angleact();
        auto forceact_msg = inspire_hand_interface::msg::Forceact();
        auto current_msg = inspire_hand_interface::msg::Current();
        auto error_msg = inspire_hand_interface::msg::Error();
        auto temp_msg = inspire_hand_interface::msg::Temp();

        auto gpos_act = send_hand_packet(1, getpos_act);
        for(int i=0;i<6;i++)
            posact_msg.pos[i] = gpos_act[i];
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%d %d %d %d %d %d'", posact_msg.pos[0], posact_msg.pos[1], posact_msg.pos[2], posact_msg.pos[3], posact_msg.pos[4], posact_msg.pos[5]);
        get_posact->publish(posact_msg);

        auto gangle_act = send_hand_packet(1, getangle_act);
        for(int i=0;i<6;i++)
            angleact_msg.angle[i] = gangle_act[i];
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%d %d %d %d %d %d'", angleact_msg.angle[0], angleact_msg.angle[1], angleact_msg.angle[2], angleact_msg.angle[3], angleact_msg.angle[4], angleact_msg.angle[5]);
        get_angleact->publish(angleact_msg);

        auto gforce_act = send_hand_packet(1, getforce_act);
        for(int i=0;i<6;i++)
            forceact_msg.force[i] = gforce_act[i];
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%d %d %d %d %d %d'", forceact_msg.force[0], forceact_msg.force[1], forceact_msg.force[2], forceact_msg.force[3], forceact_msg.force[4], forceact_msg.force[5]);
        get_forceact->publish(forceact_msg);

        auto gcurrent = send_hand_packet(1, getcurrent);
        for(int i=0;i<6;i++)
            current_msg.current[i] = gcurrent[i];
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%d %d %d %d %d %d'", current_msg.current[0], current_msg.current[1], current_msg.current[2], current_msg.current[3], current_msg.current[4], current_msg.current[5]);
        get_current->publish(current_msg);

        auto gerror = send_hand_packet(1, geterror);
        for(int i=0;i<6;i++)
            error_msg.error[i] = gerror[i];
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%d %d %d %d %d %d'", error_msg.error[0], error_msg.error[1], error_msg.error[2], error_msg.error[3], error_msg.error[4], error_msg.error[5]);
        get_error->publish(error_msg);

        auto gtemp = send_hand_packet(1, gettemp);
        for(int i=0;i<6;i++)
            temp_msg.temp[i] = gtemp[i];
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%d %d %d %d %d %d'", temp_msg.temp[0], temp_msg.temp[1], temp_msg.temp[2], temp_msg.temp[3], temp_msg.temp[4], temp_msg.temp[5]);
        get_temp->publish(temp_msg);
    }

    rclcpp::Publisher<inspire_hand_interface::msg::Posact>::SharedPtr get_posact;
    rclcpp::Publisher<inspire_hand_interface::msg::Angleact>::SharedPtr get_angleact;
    rclcpp::Publisher<inspire_hand_interface::msg::Forceact>::SharedPtr get_forceact;
    rclcpp::Publisher<inspire_hand_interface::msg::Current>::SharedPtr get_current;
    rclcpp::Publisher<inspire_hand_interface::msg::Error>::SharedPtr get_error;
    rclcpp::Publisher<inspire_hand_interface::msg::Temp>::SharedPtr get_temp;
    rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    ros_ser.setPort("/dev/ttyUSB0");
    ros_ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ros_ser.setTimeout(to);
    try
    {
        ros_ser.open();
    }
    catch(serial::IOException &e)
    {
        std::cout<<"serial unable to open"<<std::endl;
        return -1;
    }
    if(ros_ser.isOpen())
    {
        std::cout<<"serial open success"<<std::endl;
    }
    else
    {
        return -1;
    }

    rclcpp::spin(std::make_shared<ParamPublisher>());
    rclcpp::shutdown();
    return 0;
}