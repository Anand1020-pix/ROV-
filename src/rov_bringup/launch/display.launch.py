from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_file = PathJoinSubstitution([
        FindPackageShare('rov_description'), 'urdf', 'rov.urdf.xacro'
    ])
    robot_description = Command(['xacro ', xacro_file, ' use_sim:=false'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': use_sim_time}], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': use_sim_time}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('rov_description'), 'rviz', 'rov.rviz'])],
             parameters=[{'use_sim_time': use_sim_time}], output='screen'),
    ])
