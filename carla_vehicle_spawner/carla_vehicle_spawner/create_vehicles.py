import carla
import random
import time
import rclpy
import os
from rclpy.node import Node

class VehicleSpawner(Node):
    def __init__(self):
        super().__init__('vehicle_spawner')

        # Fetch ROS_DOMAIN_ID and assign the CARLA port dynamically
        ros_domain_id = int(os.getenv('ROS_DOMAIN_ID', 0))
        carla_port = 2000 + ros_domain_id
        
        self.client = carla.Client('localhost', carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.spawn_vehicles()

    def spawn_vehicles(self):
        try:
            vehicle_blueprints = self.world.get_blueprint_library().filter('vehicle.*')
            spawn_points = self.world.get_map().get_spawn_points()

            # Infinite loop to continuously spawn vehicles until interrupted
            while rclpy.ok():
                blueprint = random.choice(vehicle_blueprints)
                spawn_point = random.choice(spawn_points)
                vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
                
                if vehicle:
                    self.get_logger().info(f'Spawned {vehicle.type_id} at {spawn_point.location}')
                    
                    # Optionally activate brake lights randomly
                    if random.choice([True, False]):
                        vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Brake))
                        self.get_logger().info(f'Brake lights ON for {vehicle.type_id}')
                    else:
                        self.get_logger().info(f'Brake lights OFF for {vehicle.type_id}')
                    
                time.sleep(1)  # Control the spawn rate

        except Exception as e:
            self.get_logger().error(f"Error spawning vehicles: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VehicleSpawner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Vehicle spawning stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
