#ifndef GRIPPER_CLIENT_HPP
#define GRIPPER_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include "gripper_srv/srv/gripper_service.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

namespace gripper_client {

  /**
   * @brief Sends a gripper command to the gripper service.
   *
   * @param node The ROS 2 node.
   * @param client The gripper service client.
   * @param position The desired gripper position (0-255).
   */
  // void send_gripper_command(rclcpp::Node::SharedPtr node,
  //                           rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr client,
  //                           int position);
  bool send_gripper_command(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr client,
    int position);

  /**
   * @brief Initializes the gripper client library.
   *
   * @param node_name The name of the ROS 2 node.
   * @return A shared pointer to the created node.
   */
  rclcpp::Node::SharedPtr create_gripper_client_node(const std::string& node_name);

  /**
   * @brief Creates a gripper service client.
   *
   * @param node The ROS 2 node.
   * @param service_name The name of the gripper service.
   * @return A shared pointer to the created client.
   */
  rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr create_gripper_service_client(rclcpp::Node::SharedPtr node, const std::string& service_name);
} // namespace gripper_client

#endif 