// #include <rclcpp/rclcpp.hpp>
// #include "gripper_client.hpp"
// #include <chrono>
// #include <std_msgs/msg/string.hpp>

// using namespace std::chrono_literals;

// class GripperClientNode : public rclcpp::Node
// {
// public:
//   GripperClientNode()
//   : Node("gripper_client_node")
//   {
//     // Gripper 서비스 클라이언트 생성
//     client_ = gripper_client::create_gripper_service_client(this->shared_from_this(), "gripper_service");

//     // 테스트용 Publisher 생성 (optional)
//     publisher_ = this->create_publisher<std_msgs::msg::String>("gripper_status", 10);

//     // Gripper 명령을 주기적으로 보내는 타이머 생성 (optional)
//     timer_ = this->create_wall_timer(
//       5s,  // 5초마다 실행
//       std::bind(&GripperClientNode::send_gripper_command, this, 0)); // 초기 위치 0으로 설정

//     timer2_ = this->create_wall_timer(
//       10s,  // 10초마다 실행
//       std::bind(&GripperClientNode::send_gripper_command, this, 255)); // 두 번째 위치 255로 설정
//   }

// private:
//   void send_gripper_command(int position)
//   {
//     // Gripper 명령 전송
//     gripper_client::send_gripper_command(this->shared_from_this(), client_, position);

//     // Publisher를 통해 상태 메시지 발행 (optional)
//     auto message = std_msgs::msg::String();
//     message.data = "Gripper command sent to position: " + std::to_string(position);
//     RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//     publisher_->publish(message);
//   }

//   rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr client_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::TimerBase::SharedPtr timer2_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<GripperClientNode>());
//   rclcpp::shutdown();
//   return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "gripper_srv/srv/gripper_service.hpp"

using namespace std::chrono_literals;

class GripperClientNode : public rclcpp::Node
{
public:
  GripperClientNode()
  : Node("gripper_client_node")
  {
    client_ = this->create_client<gripper_srv::srv::GripperService>("gripper_service");
  }

private:
  void send_gripper_command(rclcpp::Node::SharedPtr node,
                            rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr client,
                            int position)
  {
    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
    request->position = position;  // 원하는 그리퍼 position
    request->speed = 255;
    request->force = 255;

    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for gripper_service...");
      return;
    }

    auto future = client_->async_send_request(request);

    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

    if (status == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Gripper Response: %s", future.get()->response.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call gripper_service");
    }

    // 요청 한 번만 보내고 종료
    rclcpp::shutdown();
  }

  rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperClientNode>());
  rclcpp::shutdown();
  return 0;
}