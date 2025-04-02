# adapted from 
# https://github.com/ros/urdf_sim_tutorial/tree/ros2

# Software License Agreement (BSD License 2.0)
#
# Copyright (c) 2023, Metro Robots
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Metro Robots nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable, SetEnvironmentVariable, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # arguments
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )
    ns_arg = DeclareLaunchArgument('namespace', description='The namespace in which ', 
                                        default_value='tiago_description')
    package_arg = DeclareLaunchArgument('urdf_package', description='The package where the robot description is located', 
                                        default_value='tiago_description')
    model_arg = DeclareLaunchArgument('urdf_package_path',description='The path to the robot description relative to the package root',
                                      default_value='robots/tiago.urdf.xacro')
    
    robot_ns = LaunchConfiguration('namespace')
    set_sim_time = SetLaunchConfiguration("use_sim_time", "True")

    # TODO should be GZ_SIM_RESOURCE_PATH, but seems not to read from it? yields error
    set_env_vars_resources = GroupAction([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH',
                               value=FindPackageShare('tiago_description')),
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH',
                                value=FindPackageShare('pmb2_description')),
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH',
                                value=FindPackageShare('pal_hey5_description')),
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH',
                                value=FindPackageShare('pal_gripper_description')),
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH',
                                value=FindPackageShare('pal_robotiq_description')),
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH',
                                value=FindPackageShare('omni_base_description'))
    ])

    # launch world
    # TODO path to world? 
    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'world': './src/robethichor/worlds/base_world.sdf',
        }.items(),
    )
    client_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py']),
        launch_arguments={
            'world': './src/robethichor/worlds/base_world.sdf',
        }.items(),
    )

    # publish robot description
    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
    )

    # spawn robot in gazebo
    urdf_spawner_node = Node(
        # package='ros_gz_sim',
        # executable='create',
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'robot', '-unpause'],
        output='screen',
    )
    
    # controllers for robot model 
    bringup_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('tiago_bringup'), 'launch', 'tiago_bringup.launch.py']),
        launch_arguments={
            'arm_type': 'no-arm',
            'is_public_sim': 'True',
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            'world_name': 'base_world'}.items()
    )

    # navigation 
    navigation_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('tiago_2dnav'), 'launch', 'tiago_nav_bringup.launch.py']),
        launch_arguments={
            'is_public_sim': 'True',
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            'slam': 'True',
            'advanced_navigation': 'True'}.items()
    )
    # navigation_launch_py = IncludeLaunchDescription(
    #     PathJoinSubstitution([FindPackageShare('tiago_advanced_2dnav'), 'launch', 'tiago_advanced_nav_bringup.launch.py']),
    #     launch_arguments={}.items()
    # )

    # moveit 
    moveit_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('tiago_moveit_config'), 'launch', 'move_group.launch.py']),
        launch_arguments={
            'arm_type': 'no-arm',
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            'tuck_arm': 'False'}.items()
    )


    # launch description with all actions created above
    ld = LaunchDescription([
        # PushRosNamespace(robot_ns),
        # set_env_vars_resources,
        set_sim_time,
        gui_arg,
        package_arg,
        model_arg,
        empty_world_launch,
        client_launch,
        # navigation_launch_py,
        moveit_launch_py,
        description_launch_py,
        urdf_spawner_node,
        bringup_launch_py,
    ])

    # ld.add_action(Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    # ))


    return ld
