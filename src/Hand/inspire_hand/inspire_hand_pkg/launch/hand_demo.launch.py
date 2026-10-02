from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 런치 아규먼트 정의 (기본값 포함)
    port_arg = DeclareLaunchArgument('port', default_value='/dev/KCCLI', description='Serial port')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200', description='Baud rate')

    # 런치 파라미터를 변수로
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')

    return LaunchDescription([
        port_arg,
        baud_arg,

        # # USB 디바이스에 대한 권한 설정
        # ExecuteProcess(
        #     cmd=['chmod', '777', port],
        #     shell=True,
        #     output='screen'
        # ),

        Node(
            package='inspire_hand_pkg',
            executable='Hand_Control',
            name='Hand_Control',
            output='screen',
            parameters=[{'port': port, 'baud': baud}],
        ),
        Node(
            package='inspire_hand_pkg',
            executable='Tactile_data_pub',
            name='Tactile_data_pub',
            output='screen',
            parameters=[],
        ),
        # Node(
        #     package='inspire_hand_pkg',
        #     executable='get_tactile_index',
        #     name='get_tactile_index',
        #     output='screen',
        #     parameters=[],
        # ),
    ])