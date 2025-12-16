from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command,LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()
    package_name = 'drone_description'
    my_robot_desc = FindPackageShare(package_name)

    default_model_path = PathJoinSubstitution([my_robot_desc,'urdf','drone.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([my_robot_desc, 'rviz', 'config_robot.rviz'])
    
    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=(default_model_path),
                                        description='Path to robot urdf file relative to package'))


    robot_description_content = ParameterValue(Command([
        'xacro', ' ', default_model_path, ' ',
    ]), value_type=str)

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content}])
    ld.add_action(start_robot_state_publisher_cmd)

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': package_name,
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))

    return ld