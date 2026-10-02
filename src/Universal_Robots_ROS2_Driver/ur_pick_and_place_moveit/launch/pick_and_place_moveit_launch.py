import launch
import os
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            # PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            PathJoinSubstitution([FindPackageShare('dual_arm'), 'urdf', 'dual_arm.xacro']),
            " ",
            "robot_ip:=192.168.1.101",
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
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur3e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )


    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic

def generate_launch_description():
    # 1. Launch arguments 선언
    gripper_robot_ip_argument = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.0.5",  # 기본 IP 주소
        description="Robot IP address",
    )

    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_pick_and_place_moveit"), "config", "kinematics.yaml"])

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

    # demo_node = Node(
    #     package="ur_pick_and_place_moveit",
    #     executable="ur_pick_and_place_moveit",
    #     name="ur_pick_and_place_moveit",
    #     output="screen",
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #     ],
    # )

    # demo_node = Node(
    #     package="ur_pick_and_place_moveit",
    #     executable="ur_pick",
    #     name="ur_pick",
    #     output="screen",
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         robot_description_kinematics,
    #     ],
    # )

    demo_node = Node(
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

    return launch.LaunchDescription([
        # 4. Launch arguments를 LaunchDescription에 추가
        # gripper_robot_ip_argument,
        demo_node,
        # include_gripper_launch,
    ])
    
# def generate_launch_description():
#     # generate_common_hybrid_launch_description() returns a list of nodes to launch
#     robot_description = get_robot_description()
#     robot_description_semantic = get_robot_description_semantic()
#     demo_node = Node(
#         package="ur_pick_and_place_moveit",
#         executable="ur_pick_and_place_moveit",
#         name="ur_pick_and_place_moveit",
#         output="screen",
#         parameters=[
#             robot_description,
#             robot_description_semantic,
#         ],
#     )

#     return launch.LaunchDescription([demo_node])
