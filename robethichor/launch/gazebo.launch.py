# based on tiago simulation package: 
# https://github.com/pal-robotics/tiago_simulation

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, SetLaunchConfiguration, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from os import environ, pathsep, path
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def generate_launch_description():
    # arguments
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )

    set_sim_time = SetLaunchConfiguration("use_sim_time", "True") 

    # launch gazebo
    world_path = PathJoinSubstitution([get_package_prefix('robethichor'),'..','..','src','robethichor', 'worlds', 'two_rooms_expanded.world'])

    # TODO umstrukturieren?
    packages = ['tiago_description', 'pmb2_description',
                'pal_hey5_description', 'pal_gripper_description',
                'pal_robotiq_description', 'omni_base_description',
                'pal_urdf_utils']

    model_paths = ""
    for package_name in packages:
        if model_paths != "":
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = path.join(package_path, "share")

        model_paths += model_path

    pkg_path = get_package_share_directory('pal_gazebo_worlds')
    model_paths = model_paths + pathsep + path.join(pkg_path, 'models')

    if 'GAZEBO_MODEL_PATH' in environ:
        model_paths = environ['GAZEBO_MODEL_PATH'] + pathsep + model_paths
    gazebo_model_path_env_var = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', model_paths)

    resource_path = pkg_path
    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_path += pathsep+environ['GAZEBO_RESOURCE_PATH']
    gazebo_resource_path_env_var = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH', resource_path)


    # ressource_paths = []
    # model_paths = []
    # for package_name in packages:
    #     package_share = FindPackageShare(package_name)
    #     ressource_paths.append(package_share)
    #     ressource_paths.append(PathJoinSubstitution([FindPackageShare(package_name),"models"]))


    # gazebo_model_path_env_var = SetEnvironmentVariable(
    #     'GAZEBO_MODEL_PATH', model_paths)
    # gazebo_resource_path_env_var = SetEnvironmentVariable(
    #     'GAZEBO_RESOURCE_PATH', ressource_paths)
    
    gz_server_launch = ExecuteProcess(cmd=[
        'gzserver', '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so', world_path,
        '--ros-args'], output='screen')
    
    gz_client_launch = ExecuteProcess(
        cmd=['gzclient'], output='screen')

    # spawn robot in gazebo
    urdf_spawner = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('tiago_gazebo'), 'launch', 'robot_spawn.launch.py']),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": 'robot',
            "base_type": 'pmb2'}.items()
    )

    # controllers for robot model 
    bringup_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('tiago_bringup'), 'launch', 'tiago_bringup.launch.py']),
        launch_arguments={
            "arm_motor_model": 'parker',
            "laser_model": 'sick-551',
            "camera_model": 'orbbec-astra',
            "base_type": 'pmb2',
            "wrist_model": 'wrist-2017',
            "ft_sensor": 'schunk-ft',
            "end_effector": 'pal-gripper',
            "has_screen": 'False',
            'arm_type': 'no-arm', # tiago-arm, no-arm
            'is_public_sim': 'True',
            "use_sim_time": LaunchConfiguration("use_sim_time")}.items()
    )

    # navigation 
    params_file = [FindPackageShare('pmb2_2dnav'), '/config', '/nav_public_sim.yaml']
    map_path = PathJoinSubstitution([get_package_prefix('robethichor'),'..','..','src','robethichor', 'worlds', 'two_rooms_expanded', 'map.yaml'])
    rviz_config_file = PathJoinSubstitution([get_package_prefix('robethichor'),'..','..','src','robethichor', 'config', 'setup.rviz'])
    
    navigation_bringup_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            'slam': 'True',
            'params_file': params_file,
            'map': map_path}.items()
    )

    navigation_localization_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py']),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            'params_file': params_file,
            'map': map_path}.items()
    )

    navigation_slam_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'slam_launch.py']),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            'params_file': params_file}.items()
    )

    navigation_rviz_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'rviz_launch.py']),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rviz_config": rviz_config_file}.items()
    )

    # moveit 
    moveit_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('tiago_moveit_config'), 'launch', 'move_group.launch.py']),
        launch_arguments={
            "robot_name": 'robot',
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            # "namespace": launch_args.namespace,
            "base_type": 'pmb2',
            "arm_type": 'no-arm', # tiago-arm no-arm
            "end_effector": 'pal-gripper',
            "ft_sensor": 'schunk-ft',}.items()
    )

    tuck_arm = Node(package='tiago_gazebo',
                    executable='tuck_arm.py',
                    name='arm_tucker',
                    emulate_tty=True,
                    output='both')


    # launch description with all actions created above
    ld = LaunchDescription([
        set_sim_time,
        gui_arg,
        gazebo_model_path_env_var,
        # gazebo_resource_path_env_var,
        gz_server_launch,
        gz_client_launch,
        urdf_spawner,
        bringup_launch_py,
        moveit_launch_py,
        navigation_bringup_py,
        navigation_localization_py,
        navigation_slam_py,
        navigation_rviz_py,
        # tuck_arm,
    ])

    return ld
