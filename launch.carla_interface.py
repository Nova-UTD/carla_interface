from os import name, path, environ

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory


def generate_launch_description():

    carla_bridge_official = Node(
        package='carla_ros_bridge',
        executable='bridge',
        name='carla_ros_bridge',
        parameters=[
            {'host': 'localhost'},
            {'port': 2000}, # + int(environ['ROS_DOMAIN_ID'])},
            {'synchronous_mode': True},
            {'town': 'Town02'},
            {'register_all_sensors': False},
            {'ego_vehicle_role_name': 'hero'},
            {'timeout': 30.0},
            {'fixed_delta_seconds': 0.2}
        ],
        remappings=[
            ('/carla/hero/lidar', '/lidar'),
            ('/carla/hero/rgb_center/image', '/cameras/camera0')
        ]
    )

    carla_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'carla_spawn_objects'), '/carla_spawn_objects.launch.py']),
        launch_arguments={
            'objects_definition_file': '/carla_interface/data/carla_objects.json'}.items(),
    ) # spawn_point_ego_vehicle

    carla_leaderboard_liaison = Node(
        package='carla_leaderboard',
        executable='liaison_node',
        parameters=[]
    )
    
    carla_lidar_processor = Node(
        package='carla_interface',
        executable='carla_lidar_processing_node'
    )

    route_reader = Node(
        package='carla_interface',
        executable='route_reader_node'
    )
    
    carla_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + '/carla_interface/data/carla.rviz']
    )

    carla_urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[path.join("/carla_interface/data", "carla.urdf")]
    )

    carla_vehicle_control = Node(
        package='carla_interface',
        executable='carla_vehicle_control_node'
    )

    return LaunchDescription([
        carla_bridge_official,
        carla_spawner,
        carla_urdf_publisher,
        carla_lidar_processor,
        carla_vehicle_control,
        carla_rviz,
    ])
