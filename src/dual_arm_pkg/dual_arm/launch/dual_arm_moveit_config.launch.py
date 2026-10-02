# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from ament_index_python.packages import get_package_share_directory
import yaml
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    rb_robot_ip = LaunchConfiguration("rb_robot_ip")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("dual_arm"), "config", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("dual_arm"), "config", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("dual_arm"), "config", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("dual_arm"), "config", "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            # PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            PathJoinSubstitution([FindPackageShare('dual_arm'), 'urdf', 'dual_arm.xacro']),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
            " rb_robot_ip:=",
            rb_robot_ip,
            " ",
        ]
    )
    # robot_description = {"robot_description": robot_description_content}
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # MoveIt Configuration
    # robot_description_semantic_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("dual_arm"), "srdf", "dual_arm.srdf"]
    #         ),
    #         " ",
    #         # "name:=",
    #         # # Also ur_type parameter could be used but then the planning group names in yaml
    #         # # configs has to be updated!
    #         # "ur",
    #         # " ",
    #         # "prefix:=",
    #         # prefix,
    #         # " ",
    #     ]
    # )
    # robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    def load_file(package_name, file_path):
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)

        with open(absolute_file_path, 'r') as file:
            return file.read()

    def load_yaml(package_name, file_path):
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)

        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)

    robot_description_semantic = {"robot_description_semantic": load_file("dual_arm", "srdf/dual_arm.srdf")}


    publish_robot_description_semantic = {
        "publish_robot_description_semantic": _publish_robot_description_semantic
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("dual_arm"), "config", "kinematics.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("dual_arm", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("dual_arm", "config/controllers.yaml")
    # the scaled_joint_trajectory_controller does not work on fake hardware
    change_controllers = context.perform_substitution(use_sim_time)
    if change_controllers == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
        # Execution time monitoring can be incompatible with the scaled JTC
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dual_arm"), "rviz", "dualarm_.rviz"]
        # [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
        # [FindPackageShare("dual_arm"), "rviz", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # Servo node for realtime control
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        output="screen",
    )
    # rb_servo_yaml = load_yaml("ur_moveit_config", "config/rb_servo.yaml")
    # rb_servo_params = {"moveit_servo": rb_servo_yaml}
    # servo_node = Node(
    #     package="moveit_servo",
    #     condition=IfCondition(launch_servo),
    #     executable="servo_node_main",
    #     parameters=[
    #         rb_servo_params,
    #         robot_description,
    #         robot_description_semantic,
    #         robot_description_kinematics,
    #     ],
    #     output="screen",
    # )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["-0.6", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link0"],
    )

    left_tf_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_left",
        arguments=["0.3", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
        output="log",
    )

    right_tf_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_right",
        arguments=["-0.3", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link0"],
        output="log",
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rb3_node = Node(
        package="rb3_730es_u_moveit_config",
        executable="move_rb",
        name="move_rb",
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,],
    )

    ur3e_node = Node(
        package="ur_pick_and_place_moveit",
        executable="move_ur",
        name="move_ur",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    # dualarm_node = Node(
    #     package="dual_arm",
    #     executable="dual_arm",
    #     name="dual_arm",
    #     output="screen",
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         robot_description_kinematics,
    #     ],
    # )

    dualarm_node = Node(
        package="dual_arm",
        executable="bimanual_service",
        name="dual_arm_service",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    dualarm_seq_node = Node(
        package="dual_arm",
        executable="bimanual_pnp",
        name="dual_arm_mainseq",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    Hand_node = Node(
        package="inspire_hand_pkg",
        executable="Hand_Control",
        name="Inspire_Hand_Control",
        output="screen",
    )


    bridge_node = Node(
        package="dual_arm",
        executable="kc_ros_zmq_bridge.py",
        name="dual_arm_bridge",
        output="screen",
    )

    # 1. Launch arguments 선언
    gripper_robot_ip_argument = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.0.5",  # 기본 IP 주소
        description="Robot IP address",
    )

    # gripper 패키지의 런치 파일 경로 찾기
    gripper_package_name = 'robotiq_hande_ros2_driver'
    gripper_package_dir = get_package_share_directory(gripper_package_name)
    gripper_launch_file_path = os.path.join(gripper_package_dir, 'launch', 'gripper_bringup.launch.py')

    # 2. LaunchConfiguration으로 robot_ip 값 가져오기
    robot_ip_config = LaunchConfiguration("robot_ip")

    # 3. IncludeLaunchDescription에 launch_arguments 추가
    include_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gripper_launch_file_path),
        launch_arguments={"robot_ip": robot_ip_config}.items(),  # robot_ip 전달
    )

    nodes_to_start = [
        move_group_node,
        rviz_node,
        servo_node,
        left_tf_pub,
        right_tf_pub,
        robot_state_publisher,
        # static_tf,
        dualarm_node,
        dualarm_seq_node,
        Hand_node,
        # bridge_node,
        # rb3_node,
        # ur3e_node,
        gripper_robot_ip_argument,
        include_gripper_launch,
        ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            description="Type/series of used UR robot.",
            # choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="True",
            description="Whether to publish the SRDF description on topic /robot_description_semantic.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )

    declared_arguments.append(
        DeclareLaunchArgument("rb_robot_ip", default_value="192.168.0.6", description="RB3 robot ip")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
