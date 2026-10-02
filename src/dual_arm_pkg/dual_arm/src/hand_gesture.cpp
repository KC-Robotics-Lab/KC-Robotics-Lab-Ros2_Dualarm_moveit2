#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <Eigen/Geometry>
#include <array>

#include "ur_pick_and_place_msgs/srv/movetcppos.hpp"
#include "rbpodo_msgs/srv/movetcppos.hpp"
#include "dual_arm_msg/srv/movetcppos.hpp"
#include "dual_arm_msg/srv/movetcppos_rb3.hpp"
#include "dual_arm_msg/srv/movetcppos_ur3.hpp"

#include "inspire_hand_interface/action/set_angle.hpp"

using SetAngle = inspire_hand_interface::action::SetAngle;
using GoalHandleSetAngle = rclcpp_action::ClientGoalHandle<SetAngle>;

uint8_t hand_id = 1;
std::array<uint16_t, 6> hand_angles;

class Handgesture : public rclcpp::Node
{
    public:
        Handgesture() : Node("Handgesture_action")
        {
            client_hand_angle = rclcpp_action::create_client<SetAngle>(this, "set_angle");

            move_seq();
        }

    private:
        rclcpp_action::Client<SetAngle>::SharedPtr client_hand_angle;
        bool goal_sent_ = false;

        void move_seq()
        {
            hand_angles = {1000, 1000, 1000, 1000, 1000, 1000};
            Hand_send_goal();
            hand_angles = {0, 1000, 1000, 1000, 1000, 1000};
            Hand_send_goal();
            hand_angles = {1000, 0, 1000, 1000, 1000, 1000};
            Hand_send_goal();
            hand_angles = {1000, 1000, 0, 1000, 1000, 1000};
            Hand_send_goal();
            hand_angles = {1000, 1000, 1000, 0, 1000, 1000};
            Hand_send_goal();
            hand_angles = {1000, 1000, 1000, 1000, 1000, 1000};
            Hand_send_goal();
            hand_angles = {1000, 1000, 1000, 1000, 0, 0};
            Hand_send_goal();
        }

        void Hand_send_goal()
        {
            // timer_->cancel();  // 한 번만 실행
        
            if (!client_hand_angle->wait_for_action_server(std::chrono::seconds(2))) {
                RCLCPP_ERROR(this->get_logger(), "SetAngle action server not available.");
                return;
            }
        
            auto goal_msg = SetAngle::Goal();
            goal_msg.hand_id = hand_id;
            goal_msg.angle0 = hand_angles[0];
            goal_msg.angle1 = hand_angles[1];
            goal_msg.angle2 = hand_angles[2];
            goal_msg.angle3 = hand_angles[3];
            goal_msg.angle4 = hand_angles[4];
            goal_msg.angle5 = hand_angles[5];
        
            RCLCPP_INFO(this->get_logger(), "Sending SetAngle goal...");
        
            auto goal_future = client_hand_angle->async_send_goal(goal_msg);
        
            // ⚠️ 기존 spin_until_future_complete(this->...) 대신 임시 executor 사용
            rclcpp::executors::SingleThreadedExecutor exec;
            exec.add_node(this->get_node_base_interface());
        
            if (exec.spin_until_future_complete(goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
                return;
            }
        
            auto goal_handle = goal_future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                return;
            }
        
            RCLCPP_INFO(this->get_logger(), "Goal accepted, waiting for result...");
        
            auto result_future = client_hand_angle->async_get_result(goal_handle);
        
            if (exec.spin_until_future_complete(result_future) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Get result call failed");
                return;
            }
        
            auto result = result_future.get();
        
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "SetAngle succeeded: %d", result.result->success);
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "SetAngle goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "SetAngle goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code from SetAngle");
                    break;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Handgesture>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}