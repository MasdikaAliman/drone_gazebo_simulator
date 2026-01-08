import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction,  IncludeLaunchDescription,  AppendEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction



def generate_launch_description():    # Get the package share directory 
    pkg_share = FindPackageShare(package='drone_description').find('drone_description')

    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'drone.urdf.xacro'])    
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share, 'rviz', 'config_robot.rviz'])


    ARGUMENTS = [
    DeclareLaunchArgument('urdf_model', default_value=urdf_path,
                          description='Absolute path to robot urdf file'),
    DeclareLaunchArgument('use_jsp', default_value='true',
                          choices=['true', 'false'],
                          description='Enable the joint state publisher'),
    DeclareLaunchArgument('jsp_gui', default_value='false',
                          choices=['true', 'false'],
                          description='Flag to enable joint_state_publisher_gui'),

    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Launch RViz2 with robot model'),
    DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config_path,
                          description='Absolute path to rviz config file'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument('use_gazebo', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to use Gazebo simulation'),
    DeclareLaunchArgument("world_file", default_value="empty.world",  description="World File path")
    ]
    
    urdf_model = LaunchConfiguration('urdf_model')
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_jsp = LaunchConfiguration('use_jsp')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    robot_description_content = ParameterValue(Command([
        'xacro ', urdf_model, ' ',
    ]), value_type=str)


    #Robot Description State
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}]
    )

    joint_state_publisher_node = Node(
        condition=IfCondition(use_jsp),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(jsp_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    rviz_node  = Node(
        condition = IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])


    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')


    gazebo_models_path_file = os.path.join(pkg_share, "models")
    set_env_vars_cmd = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gazebo_models_path_file)

    world_file = LaunchConfiguration('world_file')

    world_path = PathJoinSubstitution([
        FindPackageShare("drone_description").find("drone_description"),
        "world",
        world_file
    ])
    
    start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    condition=IfCondition(LaunchConfiguration('use_gazebo')),
    launch_arguments={
        "gz_args": PythonExpression([
            "'", world_path, " -v 4 -r'"
        ])
    }.items()
    )
    
    start_gazebo_spawner_cmd = Node(
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            "-topic", "/robot_description",
            "-name", '',
            "-x", "0.0", 
            "-y", "0.0",
            "-z", "0.0",
            "-R", "0.0",
            "-P", "0.0", 
            "-Y", "0.0", 
            '-allow_renaming', 'true',
            ]

        )



    #ROS_BRIDGE
    ros_gz_bridge_config_file_path = 'config/ros_to_gz_bridge.yaml'
    default_ros_gz_bridge_config_file = PathJoinSubstitution(
        [pkg_share, ros_gz_bridge_config_file_path]
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': default_ros_gz_bridge_config_file,
            'use_sim_time': use_sim_time
        }],
        output='screen')


    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
 
    ld.add_action(
        TimerAction(period=2.0, 
                    actions=[start_gazebo_server_cmd])
    )
    ld.add_action(
        TimerAction(period=1.0, 
                    actions=[start_gazebo_spawner_cmd])
    )
    ld.add_action(rviz_node)
    ld.add_action(start_gazebo_ros_bridge_cmd)


    
    return ld

