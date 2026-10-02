#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/u_int16_multi_array.hpp"

serial::Serial ros_ser;
unsigned char send_buffer[64] = {0};
unsigned char recv_buffer[64] = {0};
rclcpp::WallRate loop_rate(10.0);        // 10 Hz 주기로 루프 실행

class TactilemiddlePublisher : public rclcpp::Node
{
    public:
        TactilemiddlePublisher() : Node("gettactile_middle_publisher")
        {
            // 토픽 생성
            pub_tip_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/middletip", 10);
            pub_nail_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/middlenail", 10);
            pub_pad_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/middlepad", 10);

            timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&TactilemiddlePublisher::publish_message, this));
        }

    private:
        std::array<u_int16_t, 9> send_middle_tip_packet(u_int8_t hand_id, u_int16_t reg)
        {
            std::array<u_int16_t, 9> data16 = {0};
            u_int8_t check_sum = 0;

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = reg & 0xFF;
            send_buffer[6] = reg >> 8;
            send_buffer[7] = 0x12;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();                  // wait 100ms for recv data
            int count = ros_ser.available();    // data가 있는지 확인
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<9;i++)
                    {
                        data16[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "middle tip : %d %d %d %d %d %d %d %d %d",
                            data16[0],data16[1],data16[2],data16[3],data16[4],data16[5],data16[6],data16[7],data16[8]);
                }
                else
                {
                    printf("값을 읽을 수 없습니다.\n");
                }
            }
            // for(int i=0;i<9;i++)
            // {
            //     data16[i] = send_buffer[i];
            // }
            // RCLCPP_INFO(this->get_logger(), "tip : %x %x %x %x %x %x %x %x %x", data16[0],data16[1],data16[2],data16[3],data16[4]
            // ,data16[5],data16[6],data16[7],data16[8]);
            return data16;
        }

        std::array<u_int16_t, 96> send_middle_nail_packet(u_int8_t hand_id, u_int16_t reg)
        {
            std::array<u_int16_t, 96> data16 = {0};
            u_int8_t check_sum = 0;

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = reg & 0xFF;
            send_buffer[6] = reg >> 8;
            send_buffer[7] = 0x60;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();                  // wait 100ms for recv data
            int count = ros_ser.available();    // data가 있는지 확인
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<96;i++)
                    {
                        data16[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "middle nail : %d %d %d %d %d %d %d %d %d",
                            data16[0],data16[1],data16[2],data16[3],data16[4],data16[5],data16[6],data16[7],data16[8]);
                }
                else
                {
                    printf("값을 읽을 수 없습니다.\n");
                }
            }
            // for(int i=0;i<9;i++)
            // {
            //     data16[i] = send_buffer[i];
            // }
            // RCLCPP_INFO(this->get_logger(), "nail : %x %x %x %x %x %x %x %x %x", data16[0],data16[1],data16[2],data16[3],data16[4]
            // ,data16[5],data16[6],data16[7],data16[8]);
            return data16;
        }

        std::array<u_int16_t, 40> send_middle_pad_packet(u_int8_t hand_id, u_int16_t reg)
        {
            std::array<u_int16_t, 40> data16 = {0};
            u_int8_t check_sum = 0;

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = reg & 0xFF;
            send_buffer[6] = reg >> 8;
            send_buffer[7] = 0x28;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();                  // wait 100ms for recv data
            int count = ros_ser.available();    // data가 있는지 확인
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<40;i++)
                    {
                        data16[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "middle pad : %d %d %d %d %d %d %d %d %d",
                            data16[0],data16[1],data16[2],data16[3],data16[4],data16[5],data16[6],data16[7],data16[8]);
                }
                else
                {
                    printf("값을 읽을 수 없습니다.\n");
                }
            }
            // for(int i=0;i<9;i++)
            // {
            //     data16[i] = send_buffer[i];
            // }
            // RCLCPP_INFO(this->get_logger(), "pad : %x %x %x %x %x %x %x %x %x", data16[0],data16[1],data16[2],data16[3],data16[4]
            // ,data16[5],data16[6],data16[7],data16[8]);
            return data16;
        }

        void publish_message()
        {
            std_msgs::msg::UInt16MultiArray tip_msg;
            std_msgs::msg::UInt16MultiArray nail_msg;
            std_msgs::msg::UInt16MultiArray pad_msg;

            std::array<u_int16_t, 9> tip_data = send_middle_tip_packet(1, 0x0E9C);
            std::array<u_int16_t, 96> nail_data = send_middle_nail_packet(1, 0x0EAE);
            std::array<u_int16_t, 40> pad_data = send_middle_pad_packet(1, 0x0F6E);

            tip_msg.data.assign(tip_data.begin(), tip_data.end());
            nail_msg.data.assign(nail_data.begin(), nail_data.end());
            pad_msg.data.assign(pad_data.begin(), pad_data.end());

            pub_tip_->publish(tip_msg);
            pub_nail_->publish(nail_msg);
            pub_pad_->publish(pad_msg);
        }


        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_tip_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_nail_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_pad_;
        rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
#ifdef serial_def
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
#endif
    rclcpp::spin(std::make_shared<TactilemiddlePublisher>());
    rclcpp::shutdown();
    return 0;
}
