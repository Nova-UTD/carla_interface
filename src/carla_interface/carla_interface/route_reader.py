'''
Package:   carla
Filename:  route_reader.py
Author:    Will Heitman (w at heit.mn)

Reads routes from a CARLA-formatted XML file and
publishes it as a Path message
'''

import os
import rclpy
import random
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import time
import xml.etree.ElementTree as ET

from carla_msgs.msg import CarlaWorldInfo
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import carla


class RouteReaderNode(Node):

    def __init__(self):
        super().__init__('route_reader_node')
        self.route_pub = self.create_publisher(
            Path, '/planning/rough_route', qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        route_timer = self.create_timer(1.0, self.publish_route)

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)

        self.route = None
        self.clock = Clock()

        ## Need to republish the world info so that the map manager can grab it
        self.world_info_sub = self.create_subscription(
            CarlaWorldInfo, '/carla/world_info', self.world_info_cb, 10)
        self.world_info_pub = self.create_publisher(
            CarlaWorldInfo, '/carla/world_info', 10)

        self.world_info_cached = None
        self.world_info_repub_timer = self.create_timer(
            5.0, self.repub_world_info)
        ##

        #tree = ET.parse('/carla_interface/data/routes_town02.xml')
        tree = ET.parse('/carla_interface/data/routes_town02_turn.xml')
        #tree = ET.parse('/carla_interface/data/routes_town02_straight.xml')

        TARGET_ROUTE_ID: str = "0"

        route_elem = None

        for route in tree.getroot():
            if route.attrib['id'] == TARGET_ROUTE_ID:
                route_elem = route

        if route_elem is None:
            self.get_logger().error("Route not found!")
            quit(-1)

        self.route = Path()
        self.route.header.frame_id = 'map'
        self.route.header.stamp = self.clock.clock

        for position in route_elem.find('waypoints'):
            print(position.attrib)
            wp = PoseStamped()
            wp.header.frame_id = 'map'
            wp.header.stamp = self.clock.clock
            wp.pose.position.x = float(position.attrib.get('x'))
            wp.pose.position.y = float(position.attrib.get('y'))
            self.route.poses.append(wp)

    def clock_cb(self, msg: Clock):
        self.clock = msg

    def publish_route(self):
        self.route_pub.publish(self.route)

    def world_info_cb(self, msg: CarlaWorldInfo):
        if msg.opendrive == "":
            return
        self.world_info_cached = msg

    def repub_world_info(self):
        if self.world_info_cached is None:
            return
        self.world_info_pub.publish(self.world_info_cached)


def main(args=None):
    rclpy.init(args=args)

    route_reader_node = RouteReaderNode()

    rclpy.spin(route_reader_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    route_reader_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
