from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 경로 설정
    pkg_share = get_package_share_directory('inspire_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'urdf_right_with_force_sensor.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'inspirehand.rviz')

    # launch argument 선언 (원본에 model arg 있었음)
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value='',
        description='Robot model'
    )

    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # joint_state_publisher_gui 노드
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # rviz 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
