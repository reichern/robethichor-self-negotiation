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
                                        # default_value='tiago_description')
                                        default_value='turtlebot3_description')
    model_arg = DeclareLaunchArgument('urdf_package_path',description='The path to the robot description relative to the package root',
                                      default_value='turtlebot3_burger.xacro')
                                    #   default_value='robots/tiago.urdf.xacro')

    robot_ns = LaunchConfiguration('namespace')
    set_sim_time = SetLaunchConfiguration("use_sim_time", "True")

    # launch world
    # TODO path to world? 
    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'world': '../robethichor/worlds/two_rooms_expanded.sdf',
            's': 'libgazebo_ros_factory.so'
        }.items(),
    )
    client_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py']),
        launch_arguments={
            'world': '../robethichor/worlds/two_rooms_expanded.sdf',
        }.items(),
    )

    # spawn robot in gazebo
    urdf_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', 'robot_description', '-entity', 'robot' , '-unpause'],
        output='screen',
    )

    # controllers for robot model 
    bringup_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('tiago_bringup'), 'launch', 'tiago_bringup.launch.py']),
        launch_arguments={
            "arm_motor_model": 'parker',
            "laser_model": 'sick-571',
            "camera_model": 'orbbec-astra',
            "base_type": 'pmb2',
            "wrist_model": 'wrist-2017',
            "ft_sensor": 'schunk-ft',
            "end_effector": 'pal-gripper',
            "has_screen": 'False',
            'arm_type': 'tiago-arm', # tiago-arm, no-arm
            'is_public_sim': 'True',
            "use_sim_time": LaunchConfiguration("use_sim_time")}.items()
    )

    # navigation 
    navigation_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('tiago_2dnav'), 'launch', 'tiago_nav_bringup.launch.py']),
        launch_arguments={
            'is_public_sim': 'True',
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            'slam': 'True',
            "robot_name": 'robot',
            'advanced_navigation': 'False',
            'laser': 'sick-571',
            'base_type': 'pmb2',
            'world_name': 'two_rooms_expanded'}.items()
    )

    tuck_arm = Node(package='tiago_gazebo',
                    executable='tuck_arm.py',
                    name='arm_tucker',
                    # arguments=['-emulate_tty', 'True'],
                    emulate_tty=True,
                    output='both',)


    # launch description with all actions created above
    ld = LaunchDescription([
        set_sim_time,
        gui_arg,
        empty_world_launch,
        client_launch,
        urdf_spawner_node,
        bringup_launch_py,
        tuck_arm,
        navigation_launch_py,
    ])

    return ld
