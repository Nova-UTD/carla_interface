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
            {'port': 2000 + int(environ['ROS_DOMAIN_ID'])},
            {'synchronous_mode': True},
            {'town': 'Town02'},
            {'register_all_sensors': False},
            {'ego_vehicle_role_name': 'hero'},
            {'timeout': 30.0},
            {'fixed_delta_seconds': 0.2}
        ],
        remappings=[
            ('/carla/hero/rgb_front/image', '/cameras/camera0'),
            ('/carla/hero/rgb_right/image', '/cameras/camera1'),
            ('/carla/hero/rgb_back/image', '/cameras/camera2'),
            ('/carla/hero/rgb_left/image', '/cameras/camera3'),
            ('/carla/hero/rgb_front_depth/image', '/depth/camera0'),
            ('/carla/hero/imu', '/imu')
        ]
    )

    carla_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'carla_spawn_objects'), '/carla_spawn_objects.launch.py']),
        launch_arguments={
            'objects_definition_file': '/carla_interface/data/ego_config.json'}.items(),
    ) # spawn_point_ego_vehicle

    carla_leaderboard_liaison = Node(
        package='carla_leaderboard',
        executable='liaison_node',
        parameters=[]
    )

    carla_gnss_processor = Node(
        package='carla_interface',
        executable='carla_gnss_processing_node'
    )

    carla_gnss_gt_processor = Node(
        package='carla_interface',
        executable='carla_gnss_processing_node',                                                                                                                                                                                                                                                                                                                                                    
        parameters=[{'gnss_topic': 'gnss_gt'}]
    )
    
    carla_lidar_processor = Node(
        package='carla_interface',
        executable='carla_lidar_processing_node'
    )

    route_reader = Node(
        package='carla_interface',
        executable='route_reader_node'
    )

    vehicle_spawner = Node(
        package='carla_interface',
        executable='vehicle_spawner'
    )
    
    carla_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + '/carla_interface/data/carla2.rviz']
    )

    with open("/carla_interface/data/carla.urdf", 'r') as f:
        robot_desc = f.read()
    carla_urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 
                     'publish_frequency': 50.0}]
    )

    carla_manual_control = Node(
        package='carla_manual_control',
        executable='carla_manual_control',
        parameters=[{'role_name':'hero'}]
    )

    carla_vehicle_control = Node(
        package='carla_interface',
        executable='carla_vehicle_control_node'
    )

    return LaunchDescription([
        ## Setup and Configuration of CARLA:
        carla_bridge_official,      # the bridge itself, remaps topics
        carla_spawner,              # spawns and configures the ego vehicle
        carla_urdf_publisher,       # publishes the descriptor of our vehicle within ROS
        route_reader,               # publishes /planning/rough_route
        vehicle_spawner,           # spawns other vehicles within carla

        ## Sensor Processing Nodes:
        carla_gnss_processor,       # publishes /gnss/odometry and /gnss/odometry_raw
        carla_gnss_gt_processor,    # publishes /gnss_gt/odometry and /gnss_gt/odometry_raw
        carla_lidar_processor,      # publishes /lidar

        ## Control the Ego Vehicle:
        carla_vehicle_control,      # subscribes to /vehicle/control and executes commands on ego vehicle
        # carla_manual_control,      # use keyboard to drive the ego vehicle manually 
        
        # carla_rviz,                # ROS visualization, only used if not running anything else
    ])
