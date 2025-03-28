from setuptools import setup
from glob import glob
import os

package_name = 'carla_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nova at UT Dallas',
    maintainer_email='project.nova@utdallas.edu',
    description='Nodes to communicate with and configure CARLA.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_reader_node = carla_interface.route_reader:main',
            'carla_vehicle_control_node = carla_interface.carla_vehicle_control:main',
            'carla_lidar_processing_node = carla_interface.carla_lidar_processing_node:main',
            'carla_gnss_processing_node = carla_interface.carla_gnss_processing_node:main',
            'spawn_vehicles = carla_interface.spawn_vehicles:main',
        ],
    },
)
