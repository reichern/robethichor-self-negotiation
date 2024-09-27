from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_ns_arg = DeclareLaunchArgument('ns', default_value='robassistant_1', description='Robot namespace')
    connector_port_arg = DeclareLaunchArgument('port', default_value='5000', description='Port for robot connector node')

    robot_ns = LaunchConfiguration('ns')
    connector_port = LaunchConfiguration('port')

    return LaunchDescription([
        robot_ns_arg,
        connector_port_arg,

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
            Node(
                package='robethichor',
                executable='negotiation_manager_node',
                name='negotiation_manager_node'
            ),
            Node(
                package='robethichor',
                executable='mission_controller_node',
                name='mission_controller_node'
            ),
            Node(
                package='robethichor',
                executable='connector_node',
                name='connector_node',
                parameters=[{'port': connector_port}]
            ),
        ]),
    ])
