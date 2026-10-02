#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h>

#include "gripper_client.hpp"

#include "ur_pick_and_place_msgs/srv/movetcppos.hpp"

#define PI M_PI

class MoveService : public rclcpp::Node
{
    public:
        MoveService() : Node("move_service")
        {
            RCLCPP_INFO(this->get_logger(), "Starting move_to_home_service...");

            gripper_node_ = gripper_client::create_gripper_client_node("main_node");
            gripper_client_ = gripper_client::create_gripper_service_client(gripper_node_, "gripper_service");

            // MoveIt node options
            rclcpp::NodeOptions node_options;
            node_options.automatically_declare_parameters_from_overrides(true);
            move_group_node_ = rclcpp::Node::make_shared("rb3_pick_place", node_options);

            // Start thread for move_group_node
            executor_.add_node(move_group_node_);
            spinner_ = std::thread([this]() { executor_.spin(); });

            // Setup MoveGroupInterface
            // const std::string PLANNING_GROUP_ARM = "mainpulation";
            const std::string PLANNING_GROUP_ARM = "ur_manipulator";
            move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                move_group_node_, PLANNING_GROUP_ARM);

            joint_model_group_arm_ =
                move_group_arm_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

            addCollisionObjects();

            // Create Trigger service
            movetcp_service_ = this->create_service<ur_pick_and_place_msgs::srv::Movetcppos>(
                "zmk_move_to_tcppos", std::bind(&MoveService::movetcpposCallback, this,
                std::placeholders::_1, std::placeholders::_2));

            home_service_ = this->create_service<std_srvs::srv::Trigger>(
                "zmk_move_to_home",
                std::bind(&MoveService::moveToHomeCallback, this,
                std::placeholders::_1, std::placeholders::_2));

            pregrasp_service_ = this->create_service<std_srvs::srv::Trigger>(
                "zmk_move_to_pregrasp", std::bind(&MoveService::moveToPregraspCallback, this,
                std::placeholders::_1, std::placeholders::_2));

            cartesian_approach_service_ = this->create_service<std_srvs::srv::Trigger>(
                "zmk_approach_cartesian_path", std::bind(&MoveService::cartesianApproachCallback, this,
                std::placeholders::_1, std::placeholders::_2));

            cartesian_retreat_service_ = this->create_service<std_srvs::srv::Trigger>(
                "zmk_retreat_cartesian_path", std::bind(&MoveService::cartesianRetreatCallback, this,
                std::placeholders::_1, std::placeholders::_2));

            place_service_ = this->create_service<std_srvs::srv::Trigger>(
                "zmk_move_to_place", std::bind(&MoveService::moveToPlacingCallback, this,
                std::placeholders::_1, std::placeholders::_2));

            gripper_open_service_ = this->create_service<std_srvs::srv::Trigger>(
                "zmk_girp_open", std::bind(&MoveService::gripopenCallback, this,
                std::placeholders::_1, std::placeholders::_2));

            gripper_close_service_ = this->create_service<std_srvs::srv::Trigger>(
                "zmk_girp_close", std::bind(&MoveService::gripcloseCallback, this,
                std::placeholders::_1, std::placeholders::_2));
        }

        ~MoveService()
        {
            executor_.cancel();
            if (spinner_.joinable())
            spinner_.join();
        }

    private:
        double jump_threshold;
        double eef_step;
        double fraction;

        rclcpp::Node::SharedPtr gripper_node_;
        rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr gripper_client_;

        rclcpp::Node::SharedPtr move_group_node_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
        const moveit::core::JointModelGroup *joint_model_group_arm_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        rclcpp::executors::SingleThreadedExecutor executor_;
        std::thread spinner_;
        geometry_msgs::msg::Pose target_pose;

        rclcpp::Service<ur_pick_and_place_msgs::srv::Movetcppos>::SharedPtr movetcp_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pregrasp_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cartesian_approach_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cartesian_retreat_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr place_service_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_open_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_close_service_;

        void addCollisionObjects()
        {
            std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

            // ----- 지면 (ground) -----
            moveit_msgs::msg::CollisionObject ground;
            ground.header.frame_id = move_group_arm_->getPlanningFrame();  // 예: "base_link" 등
            ground.id = "ground";

            shape_msgs::msg::Plane ground_plane;
            ground_plane.coef = {0, 0, 1, 0};  // z = 0 평면

            geometry_msgs::msg::Pose ground_pose;
            ground_pose.orientation.w = 1.0;
            ground_pose.position.z = -0.01;  // 약간 밑에 위치

            ground.planes.push_back(ground_plane);
            ground.plane_poses.push_back(ground_pose);
            ground.operation = ground.ADD;
            collision_objects.push_back(ground);

            // ----- 벽 (back wall) -----
            moveit_msgs::msg::CollisionObject back_wall;
            back_wall.header.frame_id = "base_link";  // 정확한 base frame으로 수정 필요
            back_wall.id = "back_wall";

            shape_msgs::msg::Plane wall_plane;
            wall_plane.coef = {0, 1, 0, 0.15};  // y축 기준

            geometry_msgs::msg::Pose wall_pose;
            wall_pose.orientation.w = 1.0;
            wall_pose.position.x = 0.0;
            wall_pose.position.y = 0.0;
            wall_pose.position.z = 0.0;

            back_wall.planes.push_back(wall_plane);
            back_wall.plane_poses.push_back(wall_pose);
            back_wall.operation = back_wall.ADD;
            collision_objects.push_back(back_wall);

            // Apply all collision objects
            planning_scene_interface.addCollisionObjects(collision_objects);
            RCLCPP_INFO(this->get_logger(), "Added ground and back wall to the planning scene.");
        }

        void movetcpposCallback(const std::shared_ptr<ur_pick_and_place_msgs::srv::Movetcppos::Request> request,
            std::shared_ptr<ur_pick_and_place_msgs::srv::Movetcppos::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Planning TCP pose...");

            move_group_arm_->setStartStateToCurrentState();

            tf2::Quaternion orientation;
            orientation.setRPY(request->rx, request->ry, request->rz);  // Roll, Pitch, Yaw

            geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);

            // geometry_msgs::msg::Pose target_pose;
            target_pose.orientation = ros_orientation;
            target_pose.position.x = request->x;
            target_pose.position.y = request->y;
            target_pose.position.z = request->z;

            move_group_arm_->setPoseTarget(target_pose);

            // moveit::planning_interface::MoveGroupInterface::Plan plan;
            // bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            // if (success) {
            //     move_group_arm_->execute(plan);
            //     response->success = true;
            //     response->message = "Moved to pregrasp pose successfully.";
            // } else {
            //     response->success = false;
            //     response->message = "Failed to plan to pregrasp pose.";
            // }

            moveit::core::RobotState ik_state(*move_group_arm_->getCurrentState());
            kinematics::KinematicsQueryOptions options;
            moveit::planning_interface::MoveGroupInterface::Plan plan;

            // std::vector<double> ik_seed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            // ik_state.setJointGroupPositions(joint_model_group_arm_, ik_seed);

            std::vector<double> current_values;
            move_group_arm_->getCurrentState()->copyJointGroupPositions(joint_model_group_arm_, current_values);
            ik_state.setJointGroupPositions(joint_model_group_arm_, current_values);

            bool found_ik = ik_state.setFromIK(
              joint_model_group_arm_,  // JointModelGroup 포인터
              target_pose,           // geometry_msgs::msg::Pose 타입
              "tool0",          // std::string, 예: "ee_link" 혹은 "tool0"
              0.1,                    // timeout (초)
              nullptr,                // 제약 조건 콜백이 없으면 nullptr
              kinematics::KinematicsQueryOptions(), // 기본 옵션
              kinematics::KinematicsBase::IKCostFn() // 기본 cost 함수
            );

            if (found_ik)
            {
                std::vector<double> ik_joint_values;
                ik_state.copyJointGroupPositions(joint_model_group_arm_, ik_joint_values);
                move_group_arm_->setJointValueTarget(ik_joint_values);

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (success)
                {
                    // move_group_arm_->execute(plan);
                    move_group_arm_->asyncExecute(plan);
                    response->success = true;
                    response->message = "Moved to pregrasp pose successfully.";
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Planning failed!");
                    response->success = false;
                    response->message = "Failed to plan to pregrasp pose.";
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "IK solution not found.");
            }
        }

        void moveToHomeCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Planning home position...");

            moveit::core::RobotStatePtr current_state = move_group_arm_->getCurrentState(10);
            std::vector<double> joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group_arm_, joint_group_positions);

            // 원하는 홈 포지션 각도 설정
            joint_group_positions[0] = 0.0;
            joint_group_positions[1] = -PI/2;
            joint_group_positions[2] = 0.0;
            joint_group_positions[3] = -PI/2;
            joint_group_positions[4] = 0.0;
            joint_group_positions[5] = 0.0;

            move_group_arm_->setStartStateToCurrentState();
            move_group_arm_->setJointValueTarget(joint_group_positions);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                move_group_arm_->execute(my_plan);
                response->success = true;
                response->message = "Moved to home position successfully.";
                RCLCPP_INFO(this->get_logger(), "Move success.");
            } else {
                response->success = false;
                response->message = "Failed to plan to home position.";
                RCLCPP_ERROR(this->get_logger(), "Planning failed.");
            }
        }

        void moveToPregraspCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Planning pre-grasp pose...");

            move_group_arm_->setStartStateToCurrentState();

            tf2::Quaternion orientation;
            orientation.setRPY(0, -PI, 0);  // Roll, Pitch, Yaw

            geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);

            // geometry_msgs::msg::Pose target_pose;
            target_pose.orientation = ros_orientation;
            target_pose.position.x = 0.010;
            target_pose.position.y = -0.410;
            target_pose.position.z = 0.264;

            move_group_arm_->setPoseTarget(target_pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                move_group_arm_->execute(plan);
                response->success = true;
                response->message = "Moved to pregrasp pose successfully.";
            } else {
                response->success = false;
                response->message = "Failed to plan to pregrasp pose.";
            }
        }

        void cartesianApproachCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Executing Cartesian approach...");

            move_group_arm_->setStartStateToCurrentState();

            std::vector<geometry_msgs::msg::Pose> waypoints;
            // geometry_msgs::msg::Pose target_pose;
            target_pose.position.z -= 0.04;
            waypoints.push_back(target_pose);

            target_pose.position.z -= 0.04;
            waypoints.push_back(target_pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            jump_threshold = 0.0;
            eef_step = 0.01;
            fraction = move_group_arm_->computeCartesianPath(
                waypoints, eef_step, jump_threshold, trajectory);

            if (fraction > 0.9) {
                move_group_arm_->execute(trajectory);
                response->success = true;
                response->message = "Cartesian approach successful.";
            } else {
                response->success = false;
                response->message = "Cartesian approach failed with fraction: " + std::to_string(fraction);
            }
        }

        void cartesianRetreatCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Executing Cartesian retreat...");

            move_group_arm_->setStartStateToCurrentState();

            std::vector<geometry_msgs::msg::Pose> waypoints;
            // geometry_msgs::msg::Pose target_pose;
            target_pose.position.z += 0.04;
            waypoints.push_back(target_pose);

            target_pose.position.z += 0.04;
            waypoints.push_back(target_pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            jump_threshold = 0.0;
            eef_step = 0.01;
            fraction = move_group_arm_->computeCartesianPath(
                waypoints, eef_step, jump_threshold, trajectory);

            if (fraction > 0.9) {
                move_group_arm_->execute(trajectory);
                response->success = true;
                response->message = "Cartesian retreat successful.";
            } else {
                response->success = false;
                response->message = "Cartesian retreat failed with fraction: " + std::to_string(fraction);
            }
        }

        void moveToPlacingCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Planning Move to Placing Pose");

            move_group_arm_->setStartStateToCurrentState();

            std::vector<geometry_msgs::msg::Pose> carry_waypoints;
            // geometry_msgs::msg::Pose target_pose;
            target_pose.position.x += 0.200;
            target_pose.position.y += 0.100;
            carry_waypoints.push_back(target_pose);

            moveit_msgs::msg::RobotTrajectory trajectory_carry;

            jump_threshold = 0.0;
            eef_step = 0.01;

            fraction = move_group_arm_->computeCartesianPath(
                carry_waypoints, eef_step, jump_threshold, trajectory_carry);

            if (fraction > 0.9) {
                move_group_arm_->execute(trajectory_carry);
                response->success = true;
                response->message = "Moved to placing point successfully.";
            } else {
                response->success = false;
                response->message = "Failed to Moved to placing point: " + std::to_string(fraction);
            }
        }

        void gripopenCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            gripper_client::send_gripper_command(gripper_node_, gripper_client_, 0);
            response->success = true;
            response->message = "Gipper open successful.";
        }

        void gripcloseCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            gripper_client::send_gripper_command(gripper_node_, gripper_client_, 255);
            response->success = true;
            response->message = "Gipper close successful.";
        }
};

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MoveService>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveService>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

