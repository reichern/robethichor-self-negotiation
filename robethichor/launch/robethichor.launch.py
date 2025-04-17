# https://github.com/ros/urdf_launch/tree/main

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.events import matches_action
from launch_ros.actions import Node, PushRosNamespace, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # robot_ns_arg = DeclareLaunchArgument('ns', default_value='robassistant_1', description='Robot namespace')
    connector_port_arg = DeclareLaunchArgument('port', default_value='5000', description='Port for robot connector node')
    ethical_implication_file_arg = DeclareLaunchArgument('ethical_implication_file', description='Path of the ethical implication configuration file')
    disposition_activation_file_arg = DeclareLaunchArgument('disposition_activation_file', description='Path of the disposition activation configuration file')
    log_output_file_arg = DeclareLaunchArgument('log_output_file', default_value='', description='Path of the log output file')
    gazebo_arg = DeclareLaunchArgument('gazebo', default_value='False', description='Whether or not to run gazebo')

    # robot_ns = LaunchConfiguration('ns')
    connector_port = LaunchConfiguration('port')
    ethical_implication_file = LaunchConfiguration('ethical_implication_file')
    disposition_activation_file = LaunchConfiguration('disposition_activation_file')
    log_output_file = LaunchConfiguration('log_output_file')
    gazebo = LaunchConfiguration('gazebo')

    context_manager_node = LifecycleNode(
                package='robethichor',
                executable='context_manager_node',
                name='context_manager_node',
                namespace='',
            )

    ethics_manager_node = LifecycleNode(
                package='robethichor',
                executable='ethics_manager_node',
                name='ethics_manager_node',
                namespace='',
            )

    ld = LaunchDescription([
        # robot_ns_arg,
        connector_port_arg,
        ethical_implication_file_arg,
        disposition_activation_file_arg,
        log_output_file_arg,
        gazebo_arg,
        GroupAction([
            # PushRosNamespace(robot_ns),
            ethics_manager_node,
            LifecycleNode(
                package='robethichor',
                executable='ethics_manager_node',
                name='ethics_manager_node',
                namespace='interrupting_user'
            ),
            context_manager_node,
            LifecycleNode(
                package='robethichor',
                executable='context_manager_node',
                name='context_manager_node',
                namespace='interrupting_user'
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
                parameters=[{'log_output_file': log_output_file}, {'gazebo': gazebo}]
            ),
            Node(
                package='robethichor',
                executable='connector_node',
                name='connector_node',
                parameters=[{'port': connector_port}]
            ),
            Node(
                package='robethichor',
                executable='interruption_manager_node',
                name='interruption_manager_node',
            ),
        ]),
        RegisterEventHandler(
            OnProcessStart(
                target_action=context_manager_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(context_manager_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
        # When the talker reaches the 'inactive' state, make it take the 'activate' transition.
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=context_manager_node,
                start_state='configuring', goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(context_manager_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),        
        RegisterEventHandler(
            OnProcessStart(
                target_action=ethics_manager_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(ethics_manager_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
        # When the talker reaches the 'inactive' state, make it take the 'activate' transition.
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=ethics_manager_node,
                start_state='configuring', goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(ethics_manager_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),

    ])


    # ld.add_action(IncludeLaunchDescription(
    #     PathJoinSubstitution([FindPackageShare('robethichor'), 'launch', 'gazebo.launch.py']),
    #     launch_arguments={
    #         'namespace': robot_ns,
    #         'urdf_package': 'tiago_description',
    #         'urdf_package_path': PathJoinSubstitution(['robots', 'tiago.urdf.xacro'])}.items(),
    # ))

    return ld 