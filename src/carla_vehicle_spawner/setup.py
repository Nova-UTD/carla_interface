from setuptools import setup

package_name = 'carla_vehicle_spawner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='project.nova@utdallas.edu',
    description='This package randomly spawns vehicles in CARLA at different coordinates, displaying the model and vehicle ID number continuously until terminated by the user using ROS2.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_vehicles = carla_vehicle_spawner.create_vehicles:main',
        ],
    },
)
