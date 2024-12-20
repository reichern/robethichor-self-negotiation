from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_ns_arg = DeclareLaunchArgument('ns', default_value='robassistant_1', description='Robot namespace')
    connector_port_arg = DeclareLaunchArgument('port', default_value='5000', description='Port for robot connector node')
    ethical_implication_file_arg = DeclareLaunchArgument('ethical_implication_file', description='Path of the ethical implication configuration file')
    disposition_activation_file_arg = DeclareLaunchArgument('disposition_activation_file', description='Path of the disposition activation configuration file')
    log_output_file_arg = DeclareLaunchArgument('log_output_file', default_value='', description='Path of the log output file')

    robot_ns = LaunchConfiguration('ns')
    connector_port = LaunchConfiguration('port')
    ethical_implication_file = LaunchConfiguration('ethical_implication_file')
    disposition_activation_file = LaunchConfiguration('disposition_activation_file')
    log_output_file = LaunchConfiguration('log_output_file')

    return LaunchDescription([
        robot_ns_arg,
        connector_port_arg,
        ethical_implication_file_arg,
        disposition_activation_file_arg,
        log_output_file_arg,

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
                name='negotiation_manager_node',
                parameters=[{'ethical_implication_file': ethical_implication_file}, {'disposition_activation_file': disposition_activation_file}]
            ),
            Node(
                package='robethichor',
                executable='mission_controller_node',
                name='mission_controller_node',
                parameters=[{'log_output_file': log_output_file}]
            ),
            Node(
                package='robethichor',
                executable='connector_node',
                name='connector_node',
                parameters=[{'port': connector_port}]
            ),
        ]),
    ])
