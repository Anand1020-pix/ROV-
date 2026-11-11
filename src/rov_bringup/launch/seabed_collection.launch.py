from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rov_bringup")
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "simulation.launch.py"])
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [bringup_share, "worlds", "seabed_collection_world.sdf"]
            )
        }.items(),
    )

    seabed_map = Node(
        package="rov_bringup",
        executable="seabed_map_publisher",
        name="seabed_map_publisher",
        output="screen",
    )
    rviz_config = PathJoinSubstitution(
        [bringup_share, "rviz", "seabed_collection.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    map_world_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "world",
        ],
        name="map_to_gazebo_world",
        output="screen",
    )

    return LaunchDescription([simulation_launch, seabed_map, map_world_transform, rviz])
