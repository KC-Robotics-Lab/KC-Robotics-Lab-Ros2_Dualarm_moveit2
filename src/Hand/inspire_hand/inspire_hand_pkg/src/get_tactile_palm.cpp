#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/u_int16_multi_array.hpp"

serial::Serial ros_ser;
unsigned char send_buffer[64] = {0};
unsigned char recv_buffer[64] = {0};
rclcpp::WallRate loop_rate(10.0);        // 10 Hz 주기로 루프 실행

class TactilepalmPublisher : public rclcpp::Node
{
    public:
    TactilepalmPublisher() : Node("gettactile_ring_publisher")
        {
            // 토픽 생성
            pub_palm_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/palm", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&TactilepalmPublisher::publish_message, this));
        }

    private:
        std::array<u_int16_t, 112> send_palm_packet(u_int8_t hand_id, u_int16_t reg)
        {
            std::array<u_int16_t, 112> data16 = {0};
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
                    for(int i=0;i<112;i++)
                    {
                        data16[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "palm tip : %d %d %d %d %d %d %d %d %d",
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

        void publish_message()
        {
            std_msgs::msg::UInt16MultiArray palm_msg;
            std::array<u_int16_t, 112> palm_data = send_palm_packet(1, 0x1324);
            palm_msg.data.assign(palm_data.begin(), palm_data.end());
            pub_palm_->publish(palm_msg);
        }


        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_palm_;
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
    rclcpp::spin(std::make_shared<TactileringPublisher>());
    rclcpp::shutdown();
    return 0;
}
