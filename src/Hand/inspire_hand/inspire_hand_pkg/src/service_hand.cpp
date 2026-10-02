#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
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


using std::placeholders::_1;
using std::placeholders::_2;
unsigned char send_buffer[64] = {0};
unsigned char recv_buffer[64] = {0};
serial::Serial ros_ser;

// #define serial_def

class Hand_control : public rclcpp::Node
{
public:
    Hand_control() : Node("Hand_control")
    {
        RCLCPP_INFO(this->get_logger(), "인스턴스 성공");

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

    // 콕백 함수 선언

    // 1486~1497 : ANGLE_SET(W)
    void setangle_callback(const inspire_hand_interface::srv::Setangle::Request::SharedPtr request,
                               const inspire_hand_interface::srv::Setangle::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "set_angle")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

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
            for(int i = 2;i < 19;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[19] = check_sum;
            ros_ser.write(send_buffer,20);
            loop_rate.sleep();    //100ms
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);//recv 버퍼
                count = ros_ser.read(&recv_buffer[0], count); //읽은 데이터 버퍼 수
                if(recv_buffer[7] == 0x01)
                {
                    response->angle_accepted = true;
                    printf("set_angle success\n");
                }
                else
                {
                    response->angle_accepted = false;
                    printf("set_angle fail\n");
                }
            }
        }
        else
        {
            //명령 오류
            response->angle_accepted = false;
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.:%s", request->status.c_str());
        }
    }

    //1486~1497 : ANGLE_SET(R)
    void getangleset_callback(const inspire_hand_interface::srv::Getangleset::Request::SharedPtr request,
                                const inspire_hand_interface::srv::Getangleset::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "get_angleset")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0xCE;
            send_buffer[6] = 0x05;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    {
                        response->curangleset[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "get_angleset : %d %d %d %d %d %d",
                    response->curangleset[0],response->curangleset[1],response->curangleset[2],response->curangleset[3],response->curangleset[4],response->curangleset[5]);
                }
                else
                {
                    printf("get_angleset 값을 읽을 수 없습니다.\n");
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.,%s", request->status.c_str());
        }
    }

    // 1474~1485 : POS_SET(W)
    void setpos_callback(const inspire_hand_interface::srv::Setpos::Request::SharedPtr request,
                               const inspire_hand_interface::srv::Setpos::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "set_pos")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x0F;
            send_buffer[4] = 0x12;
            send_buffer[5] = 0xC2;
            send_buffer[6] = 0x05;
            send_buffer[7] = (request->pos0 & 0xFF);
            send_buffer[8] = ((request->pos0 >> 8) & 0xFF);
            send_buffer[9] = (request->pos1 & 0xFF);
            send_buffer[10] = ((request->pos1 >> 8) & 0xFF);
            send_buffer[11] = (request->pos2 & 0xFF);
            send_buffer[12] = ((request->pos2 >> 8) & 0xFF);
            send_buffer[13] = (request->pos3 & 0xFF);
            send_buffer[14] = ((request->pos3 >> 8) & 0xFF);
            send_buffer[15] = (request->pos4 & 0xFF);
            send_buffer[16] = ((request->pos4 >> 8) & 0xFF);
            send_buffer[17] = (request->pos5 & 0xFF);
            send_buffer[18] = ((request->pos5 >> 8) & 0xFF);
            for(int i = 2;i < 19;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[19] = check_sum;
            ros_ser.write(send_buffer,20);
            loop_rate.sleep();    //100ms
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[7] == 0x01)
                {
                    response->pos_accepted = true;
                    printf("set_pos success\n");
                }
                else
                {
                    response->pos_accepted = false;
                    printf("set_pos fail\n");
                }
            }
        }
        else
        {
            response->pos_accepted = false;
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.:%s", request->status.c_str());
        }
    }

    // 1474~1485 : POS_SET(R)
    void getposset_callback(const inspire_hand_interface::srv::Getposset::Request::SharedPtr request,
                                const inspire_hand_interface::srv::Getposset::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        if(request->status == "get_posset")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0xC2;
            send_buffer[6] = 0x05;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    {
                        response->curposset[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "get_posset : %d %d %d %d %d %d", 
                    response->curposset[0],response->curposset[1],response->curposset[2],response->curposset[3],response->curposset[4],response->curposset[5]);
                }
                else
                {
                    printf("포지션 set 값을 읽을 수 없습니다.\n");
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.,%s", request->status.c_str());
        }
    }

    // 1522~1533 : SPEED_SET(W)
    void setspeed_callback(const inspire_hand_interface::srv::Setspeed::Request::SharedPtr request,
                               const inspire_hand_interface::srv::Setspeed::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "set_speed")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

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
            for(int i = 2;i < 19;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[19] = check_sum;
            ros_ser.write(send_buffer,20);
            loop_rate.sleep();    //100ms
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[7] == 0x01)
                {
                    response->speed_accepted = true;
                    printf("set_speed success\n");
                }
                else
                {
                    response->speed_accepted = false;
                    printf("set_speed fail\n");
                }
            }
        }
        else
        {
            response->speed_accepted = false;
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.:%s", request->status.c_str());
        }
    }

    // 1522~1533 : SPEED_SET(R)
    void getspeedset_callback(const inspire_hand_interface::srv::Getspeedset::Request::SharedPtr request,
                                const inspire_hand_interface::srv::Getspeedset::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "get_speedset")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0xF2;
            send_buffer[6] = 0x05;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    {
                        response->curspeedset[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "get_speedset : %d %d %d %d %d %d",
                    response->curspeedset[0],response->curspeedset[1],response->curspeedset[2],response->curspeedset[3],response->curspeedset[4],response->curspeedset[5]);
                }
                else
                {
                    printf("속도 값을 읽을 수 없습니다.\n");
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다,%s", request->status.c_str());
        }
    }

    // 1498~1521 : FORCE_SET(W)
    void setforce_callback(const inspire_hand_interface::srv::Setforce::Request::SharedPtr request,
                               const inspire_hand_interface::srv::Setforce::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "set_force")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

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
            for(int i = 2;i < 19;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[19] = check_sum;
            ros_ser.write(send_buffer,20);
            loop_rate.sleep();    //100ms
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[7] == 0x01)
                {
                    response->force_accepted = true;
                    printf("set_force sucess\n");
                }
                else
                {
                    response->force_accepted = false;
                    printf("set_force fail\n");
                }
            }
        }
        else
        {
            response->force_accepted = false;
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.:%s", request->status.c_str());
        }
    }

    // 1498~1521 : FORCE_SET(R)
    void getforceset_callback(const inspire_hand_interface::srv::Getforceset::Request::SharedPtr request,
                                const inspire_hand_interface::srv::Getforceset::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "get_forceset")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0xDA;
            send_buffer[6] = 0x05;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    {
                        response->curforceset[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "get_forceset : %d %d %d %d %d %d",
                    response->curforceset[0],response->curforceset[1],response->curforceset[2],response->curforceset[3],response->curforceset[4],response->curforceset[5]);
                }
                else
                {
                    printf("get_forceset 값을 읽을 수 없습니다.\n");
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.,%s", request->status.c_str());
        }
    }

    //1546~1545 : ANGLE_ACT(R)
    void getangleact_callback(const inspire_hand_interface::srv::Getangleact::Request::SharedPtr request,
                               const inspire_hand_interface::srv::Getangleact::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "get_angleact")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());
            // response->curangleact[0] = 10;
            // response->curangleact[1] = 20;
            // response->curangleact[2] = 30;
            // response->curangleact[3] = 40;
            // response->curangleact[4] = 50;
            // response->curangleact[5] = 60;

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0x0A;
            send_buffer[6] = 0x06;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    {
                        response->curangleact[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "각도 : %d %d %d %d %d %d",
                    response->curangleact[0],response->curangleact[1],response->curangleact[2],response->curangleact[3],response->curangleact[4],response->curangleact[5]);
                }
                else
                {
                    printf("각도 값을 읽을 수 없습니다.\n");
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.,%s", request->status.c_str());
        }
    }

    // 1534~1545 : POS_ACT(R)
    void getposact_callback(const inspire_hand_interface::srv::Getposact::Request::SharedPtr request,
                               const inspire_hand_interface::srv::Getposact::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "get_posact")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0xFE;
            send_buffer[6] = 0x05;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    {
                        response->curposact[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "현재 포지션 :%d %d %d %d %d %d", 
                    response->curposact[0],response->curposact[1],response->curposact[2],response->curposact[3],response->curposact[4],response->curposact[5]);
                }
                else
                {
                    printf("포지션 값을 읽을 수 없습니다\n");
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.,%s", request->status.c_str());
        }
    }

    // 1582~1593 : FORCE_ACT(R)
    void getforceact_callback(const inspire_hand_interface::srv::Getforceact::Request::SharedPtr request,
                               const inspire_hand_interface::srv::Getforceact::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "get_forceact")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0x2E;
            send_buffer[6] = 0x06;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    {
                        response->curforceact[i] =int16_t((recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00));
                    }
                    RCLCPP_INFO(this->get_logger(), "get_forceact : %d %d %d %d %d %d",
                    response->curforceact[0],response->curforceact[1],response->curforceact[2],response->curforceact[3],response->curforceact[4],response->curforceact[5]);
                }
                else
                {
                    printf("get_forceact 값을 읽을 수 없습니다.\n");
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다,%s", request->status.c_str());
        }
    }

    // 1594~1595 : CURRENT(R)
    void getcurrentact_callback(const inspire_hand_interface::srv::Getcurrentact::Request::SharedPtr request,
                                    const inspire_hand_interface::srv::Getcurrentact::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "get_currentact")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0x3A;
            send_buffer[6] = 0x06;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    {
                        response->curcurrent[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "get_currentact : %d %d %d %d %d %d",
                    response->curcurrent[0],response->curcurrent[1],response->curcurrent[2],response->curcurrent[3],response->curcurrent[4],response->curcurrent[5]);
                }
                else
                {
                    printf("get_currentact 값을 읽을 수 없습니다.\n");
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.,%s", request->status.c_str());
        }
    }

    // 1606~1611 : ERROR(R)
    void geterror_callback(const inspire_hand_interface::srv::Geterror::Request::SharedPtr request,
                               const inspire_hand_interface::srv::Geterror::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "get_error")
        {
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0x46;
            send_buffer[6] = 0x06;
            send_buffer[7] = 0x06;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    {
                        response->error[i] = (recv_buffer[7+i] & 0xFF);
                    }
                    RCLCPP_INFO(this->get_logger(), "get_error : %d %d %d %d %d %d",
                    response->error[0],response->error[1],response->error[2],response->error[3],response->error[4],response->error[5]);
                }
                else
                {
                    printf("get_error 값을 읽을 수 없습니다.\n");
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.,%s", request->status.c_str());
        }
    }

    // 1618~1623 : TEMP(R)
    void gettemp_callback(const inspire_hand_interface::srv::Gettemp::Request::SharedPtr request,
                               const inspire_hand_interface::srv::Gettemp::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);

        if(request->status == "get_temp")
        {
            RCLCPP_INFO(this->get_logger(), "%s", request->status.c_str());

            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0x52;
            send_buffer[6] = 0x06;
            send_buffer[7] = 0x06;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();
            int count = ros_ser.available();
            if (count != 0)
            {
                std::vector<unsigned char> recv_buffer(count);
                count = ros_ser.read(&recv_buffer[0], count);
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    {
                        response->temp[i] = (recv_buffer[7+i] & 0xFF);
                    }
                    RCLCPP_INFO(this->get_logger(), "get_temp : %d %d %d %d %d %d",
                    response->temp[0],response->temp[1],response->temp[2],response->temp[3],response->temp[4],response->temp[5]);
                }
                else
                {
                    printf("get_temp 값을 읽을 수 없습니다.\n");
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "status를 읽을 수 없습니다.,%s", request->status.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    #if 1
    ros_ser.setPort("/dev/ttyUSB0");
    // ros_ser.setPort("/dev/ttyACM0");
    ros_ser.setBaudrate(115200);
    serial::Timeout to =serial::Timeout::simpleTimeout(1000);
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
    auto node = std::make_shared<Hand_control>();
    rclcpp::executors::MultiThreadedExecutor exector;
    exector.add_node(node);
    exector.spin();
    rclcpp::shutdown();
    return 0;
}

