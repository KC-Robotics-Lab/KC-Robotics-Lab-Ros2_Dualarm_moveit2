#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include <algorithm> 

#define serial_def

serial::Serial ros_ser;
unsigned char send_buffer[64] = {0};
unsigned char recv_buffer[64] = {0};
rclcpp::WallRate loop_rate(10.0);        // 10 Hz 주기로 루프 실행

class TactileindexPublisher : public rclcpp::Node
{
    public:
        TactileindexPublisher() : Node("gettactile_index_publisher")
        {
            // 토픽 생성
            pub_tip_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/indextip", 10);
            pub_nail_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/indexnail", 10);
            pub_pad_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("tactile/indexpad", 10);

            timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&TactileindexPublisher::publish_message, this));
        }

    private:
        std::vector<uint16_t> prev_tip = std::vector<uint16_t>(9, 0);
        std::vector<uint16_t> prev_nail = std::vector<uint16_t>(96, 0);
        std::vector<uint16_t> prev_pad = std::vector<uint16_t>(40, 0);

        std::array<u_int16_t, 9> send_index_tip_packet(u_int8_t hand_id, u_int16_t reg)
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
                RCLCPP_INFO(this->get_logger(), "1. tip count : %d", count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<9;i++)
                    {
                        data16[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "index tip : %d %d %d %d %d %d %d %d %d",
                            data16[0],data16[1],data16[2],data16[3],data16[4],data16[5],data16[6],data16[7],data16[8]);
                }
                else
                {
                    printf("값을 읽을 수 없습니다.\n");
                }
            }
            return data16;
        }

        std::array<u_int16_t, 96> send_index_nail_packet(u_int8_t hand_id, u_int16_t reg)
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
                RCLCPP_INFO(this->get_logger(), "2. nail count : %d", count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<96;i++)
                    {
                        data16[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "index nail : %d %d %d %d %d %d %d %d %d",
                            data16[0],data16[1],data16[2],data16[3],data16[4],data16[5],data16[6],data16[7],data16[8]);
                }
                else
                {
                    printf("값을 읽을 수 없습니다.\n");
                }
            }
            return data16;
        }

        std::array<u_int16_t, 40> send_index_pad_packet(u_int8_t hand_id, u_int16_t reg)
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
                RCLCPP_INFO(this->get_logger(), "3. tip count : %d", count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<40;i++)
                    {
                        data16[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "index pad : %d %d %d %d %d %d %d %d %d",
                            data16[0],data16[1],data16[2],data16[3],data16[4],data16[5],data16[6],data16[7],data16[8]);
                }
                else
                {
                    printf("값을 읽을 수 없습니다.\n");
                }
            }
            return data16;
        }

        void publish_message()
        {
            auto tip_data  = send_index_tip_packet(1, 0x100E);
            auto nail_data = send_index_nail_packet(1, 0x1020);
            auto pad_data  = send_index_pad_packet(1, 0x10E0);

            std_msgs::msg::UInt16MultiArray tip_msg;
            std_msgs::msg::UInt16MultiArray nail_msg;
            std_msgs::msg::UInt16MultiArray pad_msg;

            // TIP: 하나라도 변경되면 전체 발행
            if (!std::equal(tip_data.begin(), tip_data.end(), prev_tip.begin()))
            {
                std_msgs::msg::UInt16MultiArray tip_msg;
                tip_msg.data = std::vector<uint16_t>(tip_data.begin(), tip_data.end());
                pub_tip_->publish(tip_msg);
                std::copy(tip_data.begin(), tip_data.end(), prev_tip.begin());
            }

            if (!std::equal(nail_data.begin(), nail_data.end(), prev_nail.begin()))
            {
                std_msgs::msg::UInt16MultiArray nail_msg;
                nail_msg.data = std::vector<uint16_t>(nail_data.begin(), nail_data.end());
                pub_nail_->publish(nail_msg);
                std::copy(nail_data.begin(), nail_data.end(), prev_nail.begin());
            }

            if (!std::equal(pad_data.begin(), pad_data.end(), prev_pad.begin()))
            {
                std_msgs::msg::UInt16MultiArray pad_msg;
                pad_msg.data = std::vector<uint16_t>(pad_data.begin(), pad_data.end());
                pub_pad_->publish(pad_msg);
                std::copy(pad_data.begin(), pad_data.end(), prev_pad.begin());
            }

//------------------------------------변경된 데이터와 인덱스만 pub---------------------------------------
            // auto tip_data = send_index_tip_packet(1, 0x100E);
            // auto nail_data = send_index_nail_packet(1, 0x1020);
            // auto pad_data = send_index_pad_packet(1, 0x10E0);

            // std_msgs::msg::UInt16MultiArray tip_msg;
            // std_msgs::msg::UInt16MultiArray nail_msg;
            // std_msgs::msg::UInt16MultiArray pad_msg;

            // // 변화된 데이터만 추가
            // for (size_t i = 0; i < tip_data.size(); ++i)
            // {
            //     if (tip_data[i] != prev_tip[i])
            //     {
            //         tip_msg.data.push_back(i);             // index
            //         tip_msg.data.push_back(tip_data[i]);   // value
            //         prev_tip[i] = tip_data[i];
            //     }
            // }

            // for (size_t i = 0; i < nail_data.size(); ++i)
            // {
            //     if (nail_data[i] != prev_nail[i])
            //     {
            //         nail_msg.data.push_back(i);
            //         nail_msg.data.push_back(nail_data[i]);
            //         prev_nail[i] = nail_data[i];
            //     }
            // }

            // for (size_t i = 0; i < pad_data.size(); ++i)
            // {
            //     if (pad_data[i] != prev_pad[i])
            //     {
            //         pad_msg.data.push_back(i);
            //         pad_msg.data.push_back(pad_data[i]);
            //         prev_pad[i] = pad_data[i];
            //     }
            // }

            // if (!tip_msg.data.empty())  pub_tip_->publish(tip_msg);
            // if (!nail_msg.data.empty()) pub_nail_->publish(nail_msg);
            // if (!pad_msg.data.empty())  pub_pad_->publish(pad_msg);

//--------------------------------------기존 코드------------------------------------------

            // std_msgs::msg::UInt16MultiArray tip_msg;
            // std_msgs::msg::UInt16MultiArray nail_msg;
            // std_msgs::msg::UInt16MultiArray pad_msg;

            // std::array<u_int16_t, 9> tip_data = send_index_tip_packet(1, 0x100E);
            // std::array<u_int16_t, 96> nail_data = send_index_nail_packet(1, 0x1020);
            // std::array<u_int16_t, 40> pad_data = send_index_pad_packet(1, 0x10E0);

            // tip_msg.data.assign(tip_data.begin(), tip_data.end());
            // nail_msg.data.assign(nail_data.begin(), nail_data.end());
            // pad_msg.data.assign(pad_data.begin(), pad_data.end());

            // // tip_msg.layout.dim.resize(1);
            // // tip_msg.layout.dim[0].label = "tip";
            // // tip_msg.layout.dim[0].size = tip_data.size();
            // // tip_msg.layout.dim[0].stride = tip_data.size();
            // // tip_msg.layout.data_offset = 0;
            // // tip_msg.data.assign(tip_data.begin(), tip_data.end());

            // // nail_msg.layout.dim.resize(1);
            // // nail_msg.layout.dim[0].label = "nail";
            // // nail_msg.layout.dim[0].size = nail_data.size();
            // // nail_msg.layout.dim[0].stride = nail_data.size();
            // // nail_msg.layout.data_offset = 0;
            // // nail_msg.data.assign(nail_data.begin(), nail_data.end());

            // // pad_msg.layout.dim.resize(1);
            // // pad_msg.layout.dim[0].label = "pad";
            // // pad_msg.layout.dim[0].size = pad_data.size();
            // // pad_msg.layout.dim[0].stride = pad_data.size();
            // // pad_msg.layout.data_offset = 0;
            // // pad_msg.data.assign(pad_data.begin(), pad_data.end());

            // pub_tip_->publish(tip_msg);
            // pub_nail_->publish(nail_msg);
            // pub_pad_->publish(pad_msg);
//----------------------------------------------------------------------------------------
        }


        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_tip_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_nail_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_pad_;
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

    rclcpp::spin(std::make_shared<TactileindexPublisher>());
    rclcpp::shutdown();
    return 0;
}
