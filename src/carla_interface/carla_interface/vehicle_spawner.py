'''
Package:   carla_interface
Filename:  vehicle_spawner.py

Spawns and manages dynamic vehicles in the CARLA simulation using the Traffic Manager.
Configures synchronous mode settings and randomizes vehicle behaviors to create
realistic traffic scenarios.
'''

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
        ros_domain_id = int(os.getenv('ROS_DOMAIN_ID', 27))
        carla_port = 2000 + ros_domain_id
        
        self.client = carla.Client('localhost', carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        # Set world to synchronous mode to match traffic manager
        settings = self.world.get_settings()
        self.original_settings = settings
        
        # Fix the synchronous mode settings to satisfy:
        # fixed_delta_seconds <= max_substep_delta_time * max_substeps
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.025  # 10 FPS - match with carla_ros_bridge
        settings.substepping = True
        settings.max_substep_delta_time = 0.005  # 5ms
        settings.max_substeps = 5  # 5 * 0.005 = 0.025, which equals fixed_delta_seconds
        
        self.world.apply_settings(settings)
        
        # Initialize the traffic manager with port specified at creation
        self.tm_port = 8000 + ros_domain_id
        self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
        self.traffic_manager.set_synchronous_mode(True)
        
        # Store spawned vehicles to keep track of them
        self.vehicles = []
        
        # Set global traffic manager parameters
        self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        
        # Create a timer for world ticking (essential for movement in synchronous mode)
        # Match the tick rate to our fixed_delta_seconds
        self.tick_timer = self.create_timer(0.2, self.tick_world)
        
        self.spawn_vehicles()

    def tick_world(self):
        """Advance the simulation by one step - essential for movement in synchronous mode"""
        self.world.tick()

    def spawn_vehicles(self):
        try:
            # Get all vehicle blueprints
            vehicle_blueprints = self.world.get_blueprint_library().filter('vehicle.*')
            
            # Instead of filtering by wheels, filter by type to avoid attribute errors
            # Exclude bicycles and motorcycles by name pattern
            filtered_blueprints = []
            for bp in vehicle_blueprints:
                type_id = bp.id
                if not ('bicycle' in type_id.lower() or 'motorcycle' in type_id.lower()):
                    filtered_blueprints.append(bp)
            
            if not filtered_blueprints:
                self.get_logger().warning("No suitable vehicle blueprints found, using all vehicles")
                filtered_blueprints = vehicle_blueprints
                
            spawn_points = self.world.get_map().get_spawn_points()
            
            # Number of vehicles to spawn
            num_vehicles = min(75, len(spawn_points))
            self.get_logger().info(f'Attempting to spawn {num_vehicles} vehicles')
            
            # Spawn vehicles and set them to autopilot
            for i in range(num_vehicles):
                if not rclpy.ok():
                    break
                    
                blueprint = random.choice(filtered_blueprints)
                
                # Try to set the color
                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)
                
                # Try to set the driver id to ensure deterministic behavior
                if blueprint.has_attribute('driver_id'):
                    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                    blueprint.set_attribute('driver_id', driver_id)
                
                # Try to spawn the vehicle
                spawn_point = spawn_points[i] if i < len(spawn_points) else random.choice(spawn_points)
                vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
                
                if vehicle:
                    self.vehicles.append(vehicle)
                    self.get_logger().info(f'Spawned {vehicle.type_id} at {spawn_point.location}')
                    
                    # Set vehicle to autopilot using the traffic manager
                    vehicle.set_autopilot(True, self.tm_port)
                    
                    # Randomize behavior for each vehicle
                    # Set how much the vehicle will respect traffic lights (higher = more likely to run red lights)
                    self.traffic_manager.ignore_lights_percentage(vehicle, random.randint(0, 30))
                    
                    # Set how much the vehicle will ignore traffic signs (higher = more likely to ignore signs)
                    self.traffic_manager.ignore_signs_percentage(vehicle, random.randint(0, 30))
                    
                    # Set how much the vehicle will force lane changes (higher = more aggressive)
                    self.traffic_manager.force_lane_change(vehicle, random.choice([True, False]))
                    
                    # Set the vehicle's desired speed (percentage of the speed limit)
                    speed_factor = random.uniform(0.8, 1.2)  # 80% to 120% of speed limit
                    self.traffic_manager.vehicle_percentage_speed_difference(vehicle, 100 * (1 - speed_factor))
                    
                    # Set the minimum distance to other vehicles (lower = more aggressive)
                    self.traffic_manager.distance_to_leading_vehicle(vehicle, random.uniform(1.0, 4.0))
                    
                    # Set how much the vehicle will keep to its current lane (higher = more lane changes)
                    self.traffic_manager.auto_lane_change(vehicle, random.choice([True, False]))
                    
                    # Optionally activate lights
                    if random.choice([True, False]):
                        vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Position))
                
                time.sleep(0.1)  # Small delay between spawns to avoid collisions
            
            self.get_logger().info(f'Successfully spawned {len(self.vehicles)} vehicles')
            
        except Exception as e:
            self.get_logger().error(f"Error spawning vehicles: {e}")
        
    def destroy(self):
        # Clean up spawned vehicles when the node is destroyed
        self.get_logger().info(f'Destroying {len(self.vehicles)} vehicles')
        for vehicle in self.vehicles:
            if vehicle.is_alive:
                vehicle.destroy()
        self.vehicles.clear()
        
        # Restore original world settings
        if hasattr(self, 'original_settings'):
            self.world.apply_settings(self.original_settings)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleSpawner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Vehicle spawning stopped by user.')
    finally:
        # Clean up vehicles before shutting down
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
