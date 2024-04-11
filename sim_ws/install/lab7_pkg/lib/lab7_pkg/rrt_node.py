#!/usr/bin/env python3

"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import *

# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
# class Node(object):
#     def __init__(self):
#         self.x = None
#         self.y = None
#         self.parent = None
#         self.cost = None # only used in RRT*
#         self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        super().__init__('rrt_node')
        
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pose_topic = "/ego_racecar/odom"
        scan_topic = "/scan"
        drive_topic = "/drive"
        occ_topic = "/grid_view"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, 1
        )

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 1
        )

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, 1
        )

        # class attributes
        # TODO: maybe create your occupancy grid here
        self.markers_pub = self.create_publisher(
            MarkerArray, occ_topic, 10
        )
        
        self.width = 10
        self.height = 10
        self.occ_grid = [[0.0] * self.width] * self.height
        self.position = {'x': 0.0, 'y': 0.0}

    def clearMarkers(self):
        markers = MarkerArray()
        marker = Marker()
        
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ns_occmarkers"

        marker.action = Marker.DELETEALL
        
        markers.markers.append(marker)
        
        self.markers_pub.publish(markers)

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        
        # render stuff around the car
        # self.clearMarkers()

        markers = MarkerArray()
        
        rad = 1.5
        j = 0
        
        for i in np.linspace(0.0, 2 * np.pi, 10):
            x_offs, y_offs = rad * np.cos(i), rad * np.sin(i)
            
            marker = Marker()
            
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ns_occmarkers"

            marker.id = j
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = round((self.position['x'] + x_offs) / 0.05) * 0.05
            marker.pose.position.y = round((self.position['y'] + y_offs) / 0.05) * 0.05
            marker.pose.position.z = 0.0

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            markers.markers.append(marker)

            j += 1

        self.markers_pub.publish(markers)

    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        
        print("new pose")
        
        self.position = {'x': pose_msg.pose.position.x, 'y': pose_msg.pose.position.y}

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        x = None
        y = None
        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        new_node = None
        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        return path



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood

def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()