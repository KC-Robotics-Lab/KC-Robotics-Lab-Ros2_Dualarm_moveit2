#include "rclcpp/rclcpp.hpp"
#include "inspire_hand_interface/srv/getposact.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("get_posact_client");

    auto client = node->create_client<inspire_hand_interface::srv::Getposact>("Getposact");

    // 서비스가 준비될 때까지 대기
    while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(node->get_logger(), "서비스 기다리는 중...");
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "인터럽트로 종료됨");
            return 1;
        }
    }

    // 요청 생성
    auto request = std::make_shared<inspire_hand_interface::srv::Getposact::Request>();
    request->status = "get_posact";
    request->hand_id = 1;  // 사용할 손 ID (예: 1)

    // 서비스 호출
    auto result_future = client->async_send_request(request);

    // 응답 대기
    if (rclcpp::spin_until_future_complete(node, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result_future.get();
        RCLCPP_INFO(node->get_logger(), "응답 수신 완료:");
        for (int i = 0; i < 6; ++i) {
            RCLCPP_INFO(node->get_logger(), "Joint %d: %d", i, response->curposact[i]);
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "서비스 호출 실패");
    }

    rclcpp::shutdown();
    return 0;
}
