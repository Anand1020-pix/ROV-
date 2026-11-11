from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

THRUSTER_NAMES = [
    "front_left_thruster",
    "front_right_thruster",
    "rear_left_thruster",
    "rear_right_thruster",
    "vertical_front_thruster",
    "vertical_rear_thruster",
]

def generate_launch_description():
    world = LaunchConfiguration('world')
    xacro_file = PathJoinSubstitution([
        FindPackageShare('rov_description'), 'urdf', 'rov.urdf.xacro'
    ])

    robot_description = ParameterValue(
    Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
        ' ',
        'use_sim:=true'
    ]),
    value_type=str
)
    world_default = PathJoinSubstitution([
        FindPackageShare('rov_bringup'), 'worlds', 'underwater_test_world.sdf'
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r -v 3 ', world]}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    spawn = TimerAction(period=3.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'sediment_collector_rov',
                       '-topic', '/robot_description',
                       '-x', '0', '-y', '0', '-z', '-0.5'],
            output='screen'
        )
    ])

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    ]
    for name in THRUSTER_NAMES:
        joint = name + "_propeller_joint"
        topic = f"/model/sediment_collector_rov/joint/{joint}/cmd_thrust"
        # ROS Float64 publisher -> Gazebo gz.msgs.Double subscriber.
        bridge_args.append(f"{topic}@std_msgs/msg/Float64]gz.msgs.Double")

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen'
    )

    thruster_controller = Node(
        package='rov_underwater_control',
        executable='rov_thruster_controller',
        name='rov_thruster_controller',
        output='screen'
    )

    collection_server = Node(
        package='rov_underwater_control',
        executable='sediment_collection_server',
        name='sediment_collection_server',
        output='screen'
    )

    collection_sensor_sim = Node(
        package='rov_underwater_control',
        executable='collection_sensor_sim',
        name='collection_sensor_sim',
        output='screen'
    )

    sim_bridge = Node(
        package='rov_underwater_control',
        executable='rov_sim_bridge',
        name='rov_sim_bridge',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_default),
        gazebo,
        rsp,
        bridge,
        spawn,
        thruster_controller,
        collection_server,
        collection_sensor_sim,
        sim_bridge,
    ])
