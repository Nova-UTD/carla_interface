# carla_interface
Provides an interface back and forth between Navigator messages and the CARLA simulator. 
* At its core is the `ros_carla_bridge` which sends sensing data from CARLA to a ROS autonomy stack. It also receives actuator commands from the ROS stack and enacts them in CARLA on the ego vehicle. The `ros_carla_bridge` repository supports ROS2 Foxy (not Humble), which is partially why we have a dedicated Docker container for it. 
* The `ros_carla_bridge` also loads the map, configures the basic scene, and configures the ego vehicle (including which sensors it has).
* Beyond renaming some of the topics published directly by the `ros_carla_bridge` (this occurs in the launch file script), this repository also provides a few nodes to process sensing data from CARLA before presenting it to the autonomy stack. The goal of these nodes is to publish the following topics for the autonomy stack:
    * `/planning/rough_route` (published by `carla_interface/route_reader.py`)
    * `/gnss` (published by `carla_interface/carla_gnss_processing_node.py`)
    * `/lidar` (published by `carla_interface/carla_lidar_processing_node.py`)
    * `/imu` (renamed in launch file)
    * `/cameras/camera0` (front camera, renamed in launch file)
    * `/cameras/camera1` (right camera, renamed in launch file)
    * `/cameras/camera2` (back camera, renamed in launch file)
    * `/cameras/camera3` (left camera, renamed in launch file)
    * `/depth/camera0` (front camera, depth, renamed in launch file)

Usage:
1) Run the docker image: `docker compose run carla_bridge`
2) Launch: `ros2 launch launch.carla_interface.py`

## Configuring the Ego Vehicle
The ego vehicle is configured within `carla_interface/data/ego_config.json`. This file is read by the launch file. The vehicle type, spawn point, and other features of the vehicle are set in this file. Sensors along with their locations and orientations are specified here. There is a property called `autopilot` that can be made true or false to make the vehicle drive on its own.

The ego vehicle can be manually driven in CARLA by uncommenting the `carla_manual_control` node and commenting out the `carla_vehicle_control` node. This opens a dialog box that allows you to use the keyboard to navigate the vehicle when that dialog box has focus. This dialog also has an option to enable autopilot there, which is another way to make the vehicle drive on its own.

Useful for recording videos of the vehicle, there is a `/carla/hero/rgb_overhead/image` topic that is published by a camera floating above and to the left of the vehicle. 

## Spawning Vehicles and Pedestrians
We have several ways to spawn other actors in the envrionment.

* A ROS node `vehicle_spawner` will spawn vehicles
* The CARLA PythonAPI has an example code to spawn a given number of vehicles and pedestrians randomly. The `carla` directory is mounted at `/` within the docker container.
> `python3 /carla/PythonAPI/examples/generate_traffic.py --port 20XX -n 100 -w 50 --safe`
> where the port should match 2000 plus your ROS_DOMAIN_ID (set in your environment variables), `n` is the number of vehicles, and `w` is the number of walkers.  The `--safe` flag is supposed to spawn vehicles that drive safely (this doesn't seem to work perfectly).