'''
Package:   carla_interface
Filename:  carla_vehicle_control.py

Applies the commands sent from Navigator in Carla.
'''

from carla_msgs.msg import CarlaEgoVehicleControl
from rosgraph_msgs.msg import Clock
from navigator_msgs.msg import VehicleControl
import rclpy
from rclpy.node import Node


class CarlaVehicleControl(Node):
    def __init__(self):
        super().__init__('carla_vehicle_control')

        # Command publisher
        self.command_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/hero/vehicle_control_cmd',  # Must be on this topic
            10
        )

        # Command subscription from Navigator
        self.command_sub = self.create_subscription(
            VehicleControl,
            '/vehicle/control',
            self.pass_to_carla,
            10
        )

        # Clock subscription
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10
        )
        self.clock = Clock()

    def clock_cb(self, msg: Clock):
        self.clock = msg

    def pass_to_carla(self, msg: VehicleControl):

        # Form a blank command message
        # https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaEgoVehicleControl.msg
        command = CarlaEgoVehicleControl()

        # Form our header, including current time
        command.header.frame_id = 'base_link'
        command.header.stamp = self.clock.clock
        command.reverse = msg.reverse
        command.throttle = msg.throttle
        command.steer = msg.steer
        command.brake = msg.brake
        # Pubish our finished command
        self.command_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)

    carla_vehicle_control_node = CarlaVehicleControl()

    rclpy.spin(carla_vehicle_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    carla_vehicle_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
