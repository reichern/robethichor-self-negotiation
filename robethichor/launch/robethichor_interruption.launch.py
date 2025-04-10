from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_ns_arg = DeclareLaunchArgument('ns', default_value='interrupting_user', description='Robot namespace')

    robot_ns = LaunchConfiguration('ns')

    return LaunchDescription([
        robot_ns_arg,

        GroupAction([
            PushRosNamespace(robot_ns),
            Node(
                package='robethichor',
                executable='ethics_manager_node',
                name='ethics_manager_node'
            ),
            Node(
                package='robethichor',
                executable='context_manager_node',
                name='context_manager_node'
            ),
        ]),
    ])
