from os import environ

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory


def generate_launch_description():

    carla_bridge_official = Node(
        package="carla_ros_bridge",
        executable="bridge",
        name="carla_ros_bridge",
        parameters=[
            {"host": "localhost"},
            {"port": int(environ["CARLA_PORT"])},
            {"synchronous_mode": True},
            {"town": "Town03"},
            {"register_all_sensors": False},
            {"ego_vehicle_role_name": "hero"},
            {"timeout": 30.0},
            {"fixed_delta_seconds": 0.2},
        ],
        remappings=[
            # ('/carla/hero/lidar', '/lidar'),
            ("/carla/hero/rgb_center/image", "/cameras/camera0"),
            ("/carla/hero/rgb_right/image", "/cameras/camera1"),
            ("/carla/hero/rgb_back/image", "/cameras/camera2"),
            ("/carla/hero/rgb_left/image", "/cameras/camera3"),
        ],
    )

    carla_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("carla_spawn_objects"),
                "/carla_spawn_objects.launch.py",
            ]
        ),
        launch_arguments={
            "objects_definition_file": "/carla_interface/data/carla_objects.json"
        }.items(),
    )  # spawn_point_ego_vehicle

    carla_leaderboard_liaison = Node(
        package="carla_leaderboard", executable="liaison_node", parameters=[]
    )

    carla_gnss_processor = Node(
        package="carla_interface", executable="carla_gnss_processing_node"
    )

    carla_lidar_processor = Node(
        package="carla_interface", executable="carla_lidar_processing_node"
    )

    route_reader = Node(package="carla_interface", executable="route_reader_node")

    carla_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d" + "/carla_interface/data/carla2.rviz"],
    )

    with open("/carla_interface/data/carla.urdf", "r") as f:
        robot_desc = f.read()
    carla_urdf_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "publish_frequency": 50.0}],
    )

    carla_vehicle_control = Node(
        package="carla_interface", executable="carla_vehicle_control_node"
    )

    return LaunchDescription(
        [
            carla_bridge_official,
            carla_spawner,
            carla_urdf_publisher,
            carla_gnss_processor,
            carla_lidar_processor,
            carla_vehicle_control,
            # carla_rviz,
            route_reader,
        ]
    )
