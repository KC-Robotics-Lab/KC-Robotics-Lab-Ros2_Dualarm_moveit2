from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # === 먼저 실행할 UR 시퀀스 노드 ===
    ur_startup_node = Node(
        package='dual_arm',
        executable='ur_startup_node.py',
        name='ur_startup_node',
        output='screen'
    )

    # === 나중에 실행할 dual_arm_bringup.launch.py ===
    dual_arm_pkg = get_package_share_directory('dual_arm')
    dual_arm_launch_file = os.path.join(dual_arm_pkg, 'launch', 'dual_arm_bringup.launch.py')

    dual_arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dual_arm_launch_file)
    )

    # ur_startup_node가 먼저 실행되고, 10초 후 dual_arm_bringup 실행
    # delayed_dual_arm = TimerAction(
    #     period=20.0,  # 10초 후 실행 (원하면 더 늘릴 수 있음)
    #     actions=[dual_arm_bringup]
    # )

    return LaunchDescription([
        ur_startup_node,
        # delayed_dual_arm
    ])