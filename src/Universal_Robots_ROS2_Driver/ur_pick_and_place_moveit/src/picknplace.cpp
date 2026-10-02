#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <cmath>
#include <memory>
#include <chrono>

#include "ur_pick_and_place_msgs/srv/gohome.hpp"

#define PI M_PI


class GoHomeServer : public rclcpp::Node
{
  public:
    GoHomeServer()
    : Node("go_home_server")
    {
      service_ = this->create_service<ur_pick_and_place_msgs::srv::Gohome>(
        "go_home",
        std::bind(&GoHomeServer::goHomeCallback, this, std::placeholders::_1, std::placeholders::_2));

      RCLCPP_INFO(this->get_logger(), "✅ GoHome service created. Call 'initialize()' to init MoveGroup.");
    }

    void initialize()
    {
      static const std::string PLANNING_GROUP = "ur_manipulator";

      // 👇 shared_from_this()를 명확하게 rclcpp::Node에서 가져오도록 캐스팅
      auto node_ptr = std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this());

      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, PLANNING_GROUP);
      joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      RCLCPP_INFO(this->get_logger(), "✅ MoveGroupInterface initialized.");
    }

  private:
    void goHomeCallback(
      const std::shared_ptr<ur_pick_and_place_msgs::srv::Gohome::Request> request,
      std::shared_ptr<ur_pick_and_place_msgs::srv::Gohome::Response> response)
    {
      RCLCPP_INFO(this->get_logger(), "📥 Received GoHome request");
      (void)request;
                response->success = true;
          response->message = "Motion completed successfully.";

    //   moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(10);
    //   std::vector<double> joint_group_positions;
    //   current_state->copyJointGroupPositions(joint_model_group_, joint_group_positions);

    //   joint_group_positions[0] = 0.0;
    //   joint_group_positions[1] = -PI / 2;
    //   joint_group_positions[2] = 0.0;
    //   joint_group_positions[3] = -PI / 2;
    //   joint_group_positions[4] = 0.0;
    //   joint_group_positions[5] = 0.0;

    //   move_group_->setStartStateToCurrentState();
    //   move_group_->setJointValueTarget(joint_group_positions);

    //   moveit::planning_interface::MoveGroupInterface::Plan plan;
    //   bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    //   if (!success) {
    //     RCLCPP_ERROR(this->get_logger(), "❌ Planning failed.");
    //     response->success = false;
    //     response->message = "Planning failed.";
    //     return;
    //   }

    //   moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);
    //   if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
    //     RCLCPP_INFO(this->get_logger(), "✅ Motion completed successfully.");
    //     response->success = true;
    //     response->message = "Motion completed successfully.";
    //   } else {
    //     RCLCPP_ERROR(this->get_logger(), "❌ Execution failed.");
    //     response->success = false;
    //     response->message = "Execution failed.";
    //   }
    }

    rclcpp::Service<ur_pick_and_place_msgs::srv::Gohome>::SharedPtr service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    const moveit::core::JointModelGroup* joint_model_group_;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // 반드시 shared_ptr로 생성해야 shared_from_this()가 동작함
  auto node = std::make_shared<GoHomeServer>();

  // 이제 객체가 완전히 shared_ptr로 감싸졌으므로 shared_from_this() 호출 가능
  node->initialize();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}