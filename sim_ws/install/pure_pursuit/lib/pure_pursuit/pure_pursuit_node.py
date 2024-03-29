#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import csv
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker, MarkerArray

root_dir = "/home/vy/GalacticRacing/sim_ws/src"

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Define subscribers
        self.pose_sub = self.create_subscription(
            Pose, "/pf/viz/inferred_pose", self.pose_callback, 10
        )

        # Define publishers
        self.markers_pub = self.create_publisher(
            MarkerArray, "/visualization_marker_array", 10
        )

        self.markers = []
        self.path = []

    def pose_callback(self, pose_msg):
        print((pose_msg.position.x, pose_msg.position.y), pose_msg.orientation)
        # TODO: find the current waypoint to track using methods mentioned in lecture

        # TODO: transform goal point to vehicle frame of reference

        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle.
    
    def get_path(self, filename):
        # open waypoint file
        waypoint_csv = open(root_dir + '/pure_pursuit/paths/' + filename + '.csv', 'r')

        reader = csv.reader(waypoint_csv, delimiter=',')
        markers = MarkerArray()

        for idx, row in enumerate(reader):
            pos_x, pos_y = float(row[0]), float(row[1])
            q_x, q_y, q_z, q_w = float(row[2]), float(row[3]), float(row[4]), float(row[5])
            theta = float(row[6])
            vel = float(row[7])

            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ns_markers"

            marker.id = idx
            marker.type = 2
            marker.action = 0

            marker.pose.position.x = pos_x
            marker.pose.position.y = pos_y
            marker.pose.position.z = 0.1

            marker.pose.orientation.x = q_x
            marker.pose.orientation.y = q_y
            marker.pose.orientation.z = q_z
            marker.pose.orientation.w = q_w

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.markers.append(marker)

            self.path.append({
                'x': pos_x,
                'y': pos_y,
                'theta': theta,
                'speed': vel
            })

        markers.markers = self.markers

        self.markers_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    pure_pursuit_node.get_path('levine_blocked_1')
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
