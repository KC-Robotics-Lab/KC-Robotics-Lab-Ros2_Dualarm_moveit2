from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction

def print_command_fn(context):
    xacro_cmd = [
        FindExecutable(name='xacro').perform(context),
        " ",
        "name:=" + LaunchConfiguration("name").perform(context),
        " ",
        "ur_type:=" + LaunchConfiguration("ur_type").perform(context),
        " ",
        "tf_prefix:=" + LaunchConfiguration("tf_prefix").perform(context),
        " ",
        "robot_ip:=" + LaunchConfiguration("robot_ip").perform(context),
        " ",
        "rb_robot_ip:=" + LaunchConfiguration("rb_robot_ip").perform(context),
        " ",
        PathJoinSubstitution(
            [FindPackageShare('dual_arm'), 'urdf', 'dual_arm.xacro']
        ).perform(context)
    ]
    print("[XACRO COMMAND] " + "".join(xacro_cmd))
    return []

debug_action = OpaqueFunction(function=print_command_fn)

def print_params(context):
    print("[LAUNCH PARAMS]")
    print("  name:", LaunchConfiguration("name").perform(context))
    print("  ur_type:", LaunchConfiguration("ur_type").perform(context))
    print("  tf_prefix:", LaunchConfiguration("tf_prefix").perform(context))
    print("  robot_ip:", LaunchConfiguration("robot_ip").perform(context))
    print("  rb_robot_ip:", LaunchConfiguration("rb_robot_ip").perform(context))
    return []

debug_params = OpaqueFunction(function=print_params)



def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            default_value="ur3e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            description="Type/series of used UR robot.",
            default_value="left_arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='"left_"',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value='"192.168.0.5"',
            description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rb_robot_ip",
            default_value='"192.168.0.6"',
            description="IP address by which the robot can be reached."
        )
    )

    ur_type = LaunchConfiguration("ur_type")
    name = LaunchConfiguration("name")
    tf_prefix = LaunchConfiguration("tf_prefix")
    robot_ip = LaunchConfiguration("robot_ip")
    rb_robot_ip = LaunchConfiguration("rb_robot_ip")

    urdf_file = PathJoinSubstitution(
        [FindPackageShare('dual_arm'), 'urdf', 'dual_arm.xacro']
    )

    # xacro로 URDF 문자열을 런타임에 생성
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        "name:=",
        name,
        " ",
        "ur_type:=",
        ur_type,
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "robot_ip:=",
        robot_ip,
        " ",
        "rb_robot_ip:=",
        rb_robot_ip,
        " ",
        urdf_file
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    return LaunchDescription([
        *declared_arguments,
        debug_action,
        debug_params,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])

