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
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable, SetEnvironmentVariable, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # these are the arguments you can pass this launch file, for example paused:=true
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )
    package_arg = DeclareLaunchArgument('urdf_package', description='The package where the robot description is located', 
                                        default_value='tiago_description')
    model_arg = DeclareLaunchArgument('urdf_package_path',description='The path to the robot description relative to the package root',
                                      default_value='robots/tiago.urdf.xacro')

    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'g': LaunchConfiguration('gui'),
            # 'pause': 'true',
        }.items(),
    )

    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
    )

    # push robot_description to factory and spawn robot in gazebo
    urdf_spawner_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity_name', 'robot', '-z', '0.5'],
        output='screen',
    )

    # set_env_vars_resources = GroupAction([
    #     SetEnvironmentVariable(name='GAZEBO_MODEL_PATH',
    #                            value=FindPackageShare('tiago_description')),
    #     AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH',
    #                             value=FindPackageShare('pmb2_description')),
    #     AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH',
    #                             value=FindPackageShare('pal_hey5_description')),
    #     AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH',
    #                             value=FindPackageShare('pal_gripper_description')),
    #     AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH',
    #                             value=FindPackageShare('pal_robotiq_description')),
    #     AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH',
    #                             value=FindPackageShare('omni_base_description'))
    # ])
    

    ld = LaunchDescription([
        gui_arg,
        package_arg,
        model_arg,
        empty_world_launch,
        description_launch_py,
        urdf_spawner_node,
        # set_env_vars_resources
    ])

    # ld.add_action(set_env_vars_resources)

    return ld