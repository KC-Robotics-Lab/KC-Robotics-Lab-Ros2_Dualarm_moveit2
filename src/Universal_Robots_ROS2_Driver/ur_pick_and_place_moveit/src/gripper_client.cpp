#include "gripper_client.hpp"

using namespace std::chrono_literals;

namespace gripper_client
{

  void send_gripper_command(rclcpp::Node::SharedPtr node,
                            rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr client,
                            int position)
  {
    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();

    // Set request parameters (example values)
    request->position = position;
    request->speed = 255;
    request->force = 255;

    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok()) 
      {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(node->get_logger(), "waiting for service gripper_service...");
    }

    auto result = client->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Position %d Result: %s", position, result.get()->response.c_str());
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to call service gripper_service for position %d", position);
    }
  }

  rclcpp::Node::SharedPtr create_gripper_client_node(const std::string& node_name) 
  {
    return rclcpp::Node::make_shared(node_name);
  }

  rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr create_gripper_service_client(rclcpp::Node::SharedPtr node, const std::string& service_name)
  {
      return node->create_client<gripper_srv::srv::GripperService>(service_name);
  }

} 