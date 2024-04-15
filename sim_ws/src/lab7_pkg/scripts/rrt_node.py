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
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import random

# TODO: import as you need
import lab7_pkg.helper as hp
import lab7_pkg.variables as vars

# class def for tree nodes
# It's up to you if you want to use this
# class Node(object):
#     def __init__(self):
#         self.x = None
#         self.y = None
#         self.parent = None
#         self.cost = None # only used in RRT*
#         self.is_root = False

class Vertex:
    def __init__(self, position={'x': 0, 'y': 0}, parent=None, children=[], input_dir=0.0, cost=0.0):
        self.position = position
        self.parent = parent
        self.children = children
        self.input_dir = input_dir
        self.cost = cost

# class def for RRT
class RRT(Node):
    def __init__(self):
        super().__init__('rrt_node')
        
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        odom_topic = "/ego_racecar/odom"
        pose_topic = "/pf/viz/inferred_pose"
        scan_topic = "/scan"
        drive_topic = "/drive"
        occ_topic = "/grid_view"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 1
        )
        
        # self.pose_sub = self.create_subscription(
        #     PoseStamped, pose_topic, self.pose_callback, 1
        # )

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
            MarkerArray, occ_topic, 20
        )
        
        self.div_width = 50
        self.div_height = 50
        self.position = {'x': 0.0, 'y': 0.0}
        self.rotation = 0.0   # radians
        self.lookahead = 0.25 # meters
        self.direction = 0    # orientation relative to map; 0 = +x, 90 = +y, 180 = -x, 270 = -y
        
        # dictionaries : maps (x_divs, y_divs) pair to an occupied probability (0.0 < p <= 1.0),
        #   and the (x, y) coordinate of the grid square in the map's coordinate frame
        # -----
        #   x_divs : horizontal grid index from origin (+/- are directional)
        #   y_divs : vertical grid index from origin (+/- are directional)

        self.global_occ_grid = {}
        self.local_occ_grid = {}
        self.tree = []
        self.curr_goal = {
            'x': -1.0,
            'y': 0.7
        }
    
    def has_neighbor(self, key):
        x_divs_str, y_divs_str = key.strip("()").split(",")
        x_divs = int(x_divs_str)
        y_divs = int(y_divs_str)
        
        x_offs = [-1, 0, 1, -1, 1, -1, 0, 1]
        y_offs = [-1, -1, -1, 0, 0, 1, 1, 1]
        
        for i in range(8):
            query_key = f"({x_divs + x_offs[i]},{y_divs + y_offs[i]})"
            
            if query_key in self.global_occ_grid:
                return True
    
        return False

    def render_global_occ(self):
        # render all occupied grid spaces on map
        markers = MarkerArray()
        
        j = 0
        
        del_keys = []
        
        for key, val in self.global_occ_grid.items():
            # check that grid cell has an orthogonal neighbor
            if self.has_neighbor(key) and val["p"] >= 3:
                marker = Marker()
                
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "ns_occmarkers"

                marker.id = j
                marker.type = Marker.CUBE
                marker.action = Marker.MODIFY

                marker.pose.position.x = val["x"]
                marker.pose.position.y = val["y"]
                marker.pose.position.z = 0.0

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.04
                marker.scale.y = 0.04
                marker.scale.z = 0.04

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                
                markers.markers.append(marker)

                j += 1
            
            # otherwise, remove this grid cell from the occupancy grid
            else:
                del_keys.append(key)
        
        for key in del_keys:
            del self.global_occ_grid[key]

        self.markers_pub.publish(markers)
    
    def render_local_occ(self):
        # render locally occupied grid spaces on map
        markers = MarkerArray()
        
        j = 0
        
        for _, val in self.local_occ_grid.items():
            if val["p"] >= 3:
                marker = Marker()
                
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "ns_localoccmarkers"

                marker.id = j
                marker.type = Marker.CUBE
                marker.action = Marker.MODIFY

                marker.pose.position.x = val["x"]
                marker.pose.position.y = val["y"]
                marker.pose.position.z = 0.1

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.04
                marker.scale.y = 0.04
                marker.scale.z = 0.04

                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.8
                
                markers.markers.append(marker)

                j += 1

        self.markers_pub.publish(markers)
        
    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        
        ranges = scan_msg.ranges
        
        # reset local occupancy grid
        self.local_occ_grid.clear()
                
        x_width = 5
        y_width = 5
        
        # collect laser scan data for collisions around car
        for i in range(vars.RANGE_SIZE):
            r = ranges[i] + 0.01
            car_dir = self.rotation
            theta = car_dir + hp.idx_to_rad(i, vars.FOV, vars.RANGE_SIZE)
            
            x_las, y_las = 0.261 * np.cos(car_dir), 0.261 * np.sin(car_dir)
            x_offs, y_offs = x_las + r * np.cos(theta), y_las + r * np.sin(theta)
            x_divs, y_divs = round((self.position['x'] + x_offs) / 0.05), round((self.position['y'] + y_offs) / 0.05)
            
            x_divs_car, y_divs_car = round(self.position['x'] / 0.05), round(self.position['y'] / 0.05)
            
            x_pos = x_divs * 0.05
            y_pos = y_divs * 0.05
            
            # add to global occupancy grid if it doesn't already exist
            # if f"({x_divs},{y_divs})" not in self.global_occ_grid:
            #     self.global_occ_grid[f"({x_divs},{y_divs})"] = {
            #         "p": 1,
            #         "x": x_pos,
            #         "y": y_pos
            #     }
                
            #     for x in range(-x_width, x_width + 1):
            #         for y in range(-y_width, y_width + 1):
            #             x_div = x_divs + x
            #             y_div = y_divs + y
                        
            #             if ((x_div - x_divs)**2 + (y_div - y_divs)**2)**0.5 <= 5:
            #                 if (
            #                     f"({x_div},{y_div})" not in self.global_occ_grid and
            #                     abs(x_div - x_divs_car) <= self.div_width and
            #                     abs(y_div - y_divs_car) <= self.div_height
            #                 ):
            #                     self.global_occ_grid[f"({x_div},{y_div})"] = {
            #                         "p": 1,
            #                         "x": x_div * 0.05,
            #                         "y": y_div * 0.05
            #                     }
            #                 elif (
            #                     f"({x_div},{y_div})" in self.global_occ_grid and
            #                     abs(x_div - x_divs_car) <= self.div_width and
            #                     abs(y_div - y_divs_car) <= self.div_height
            #                 ):
            #                     val = self.global_occ_grid[f"({x_div},{y_div})"]
                                
            #                     self.global_occ_grid[f"({x_div},{y_div})"] = {
            #                         "p": val["p"] + 1,
            #                         "x": x_div * 0.05,
            #                         "y": y_div * 0.05
            #                     }
            # else:
            #     val = self.global_occ_grid[f"({x_divs},{y_divs})"]
                
            #     self.global_occ_grid[f"({x_divs},{y_divs})"] = {
            #         "p": val["p"] + 1,
            #         "x": x_pos,
            #         "y": y_pos
            #     }
                
            #     for x in range(-x_width, x_width + 1):
            #         for y in range(-y_width, y_width + 1):
            #             x_div = x_divs + x
            #             y_div = y_divs + y
                        
            #             if ((x_div - x_divs)**2 + (y_div - y_divs)**2)**0.5 <= 5:
            #                 if (
            #                     f"({x_div},{y_div})" not in self.global_occ_grid and
            #                     abs(x_div - x_divs_car) <= self.div_width and
            #                     abs(y_div - y_divs_car) <= self.div_height
            #                 ):
            #                     self.global_occ_grid[f"({x_div},{y_div})"] = {
            #                         "p": 1,
            #                         "x": x_div * 0.05,
            #                         "y": y_div * 0.05
            #                     }
            #                 elif (
            #                     f"({x_div},{y_div})" in self.global_occ_grid and
            #                     abs(x_div - x_divs_car) <= self.div_width and
            #                     abs(y_div - y_divs_car) <= self.div_height
            #                 ):
            #                     val = self.global_occ_grid[f"({x_div},{y_div})"]
                                
            #                     self.global_occ_grid[f"({x_div},{y_div})"] = {
            #                         "p": val["p"] + 1,
            #                         "x": x_div * 0.05,
            #                         "y": y_div * 0.05
            #                     }
            
            # add to local occupancy grid if it doesn't already exist
            # and if it is within a bounding box around the car
            if (
                f"({x_divs},{y_divs})" not in self.local_occ_grid and
                abs(x_divs - x_divs_car) <= self.div_width and
                abs(y_divs - y_divs_car) <= self.div_height
            ):
                self.local_occ_grid[f"({x_divs},{y_divs})"] = {
                    "p": 1,
                    "x": x_pos,
                    "y": y_pos
                }
                
                for x in range(-x_width, x_width + 1):
                    for y in range(-y_width, y_width + 1):
                        x_div = x_divs + x
                        y_div = y_divs + y
                        
                        if ((x_div - x_divs)**2 + (y_div - y_divs)**2)**0.5 <= 5:
                            if (
                                f"({x_div},{y_div})" not in self.local_occ_grid and
                                abs(x_div - x_divs_car) <= self.div_width and
                                abs(y_div - y_divs_car) <= self.div_height
                            ):
                                self.local_occ_grid[f"({x_div},{y_div})"] = {
                                    "p": 1,
                                    "x": x_div * 0.05,
                                    "y": y_div * 0.05
                                }
                            elif (
                                f"({x_div},{y_div})" in self.local_occ_grid and
                                abs(x_div - x_divs_car) <= self.div_width and
                                abs(y_div - y_divs_car) <= self.div_height
                            ):
                                val = self.local_occ_grid[f"({x_div},{y_div})"]
                                
                                self.local_occ_grid[f"({x_div},{y_div})"] = {
                                    "p": val["p"] + 1,
                                    "x": x_div * 0.05,
                                    "y": y_div * 0.05
                                }
                        
            elif (
                f"({x_divs},{y_divs})" in self.local_occ_grid and
                abs(x_divs - x_divs_car) <= self.div_width and
                abs(y_divs - y_divs_car) <= self.div_height
            ):
                val = self.local_occ_grid[f"({x_divs},{y_divs})"]
                
                self.local_occ_grid[f"({x_divs},{y_divs})"] = {
                    "p": val["p"] + 1,
                    "x": x_pos,
                    "y": y_pos
                }
                
                for x in range(-x_width, x_width + 1):
                    for y in range(-y_width, y_width + 1):
                        x_div = x_divs + x
                        y_div = y_divs + y
                        
                        if ((x_div - x_divs)**2 + (y_div - y_divs)**2)**0.5 <= 5:
                            if (
                                f"({x_div},{y_div})" not in self.local_occ_grid and
                                abs(x_div - x_divs_car) <= self.div_width and
                                abs(y_div - y_divs_car) <= self.div_height
                            ):
                                self.local_occ_grid[f"({x_div},{y_div})"] = {
                                    "p": 1,
                                    "x": x_div * 0.05,
                                    "y": y_div * 0.05
                                }
                            elif (
                                f"({x_div},{y_div})" in self.local_occ_grid and
                                abs(x_div - x_divs_car) <= self.div_width and
                                abs(y_div - y_divs_car) <= self.div_height
                            ):
                                val = self.local_occ_grid[f"({x_div},{y_div})"]
                                
                                self.local_occ_grid[f"({x_div},{y_div})"] = {
                                    "p": val["p"] + 1,
                                    "x": x_div * 0.05,
                                    "y": y_div * 0.05
                                }
                
        # self.render_global_occ()
        self.render_local_occ()
        
    
    def odom_callback(self, odom_msg):
        """
        The odometry callback when subscribed to ego_racecar's simulated pose
        Here is where the main RRT loop happens

        Args: 
            odom_msg (Odometry): incoming message from subscribed topic
        Returns:

        """
        
        p = odom_msg.pose.pose.position
        o = odom_msg.pose.pose.orientation
        
        self.position = {'x': p.x, 'y': p.y}
        self.rotation = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        self.direction = int(hp.rad_to_deg(round(self.rotation / hp.deg_to_rad(90.0)) * hp.deg_to_rad(90.0)))
        
        self.rrt()

    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        
        p = pose_msg.pose.position
        o = pose_msg.pose.orientation
        
        self.position = {'x': p.x, 'y': p.y}
        self.rotation = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        self.direction = int(hp.rad_to_deg(round(self.rotation / hp.deg_to_rad(90.0)) * hp.deg_to_rad(90.0)))
    
    def rrt(self):
        self.tree = [Vertex(position={'x': self.position['x'], 'y': self.position['y']})]
        
        while True:
            rand_pt = self.sample()
            nearest_idx = self.nearest(rand_pt)
            nearest_node = self.tree[nearest_idx]
            new_node = self.steer(self.tree[nearest_idx], rand_pt)
            
            if not self.check_collision(nearest_node, new_node):
                self.tree.append(new_node)
                
                if self.is_goal(new_node, self.curr_goal['x'], self.curr_goal['y']):
                    self.key_points = self.find_path(new_node)
                    break
        
        self.render_path()

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        
        switch = {
            0: {'x': (0, self.div_width), 'y': (-self.div_height, self.div_height)},
            90: {'x': (-self.div_width, self.div_width), 'y': (0, self.div_height)},
            180: {'x': (-self.div_width, 0), 'y': (-self.div_height, self.div_height)},
            270: {'x': (-self.div_width, self.div_width), 'y': (-self.div_height, 0)}
        }
        
        dir = self.direction + 360 if self.direction < 0 else self.direction
        
        x = random.randint(switch[dir]['x'][0], switch[dir]['x'][1])
        y = random.randint(switch[dir]['y'][0], switch[dir]['y'][1])
        
        x_div_car = round(self.position['x'] / 0.05)
        y_div_car = round(self.position['y'] / 0.05)
        
        return (x_div_car + x, y_div_car + y)

    def nearest(self, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        
        sample_pt = {
            'x': sampled_point[0],
            'y': sampled_point[1]
        }
        
        root_pt = {
            'x': self.tree[0].position['x'],
            'y': self.tree[0].position['y']
        }
        
        min_dist = (sample_pt['x'] - root_pt['x'])**2 + (sample_pt['y'] - root_pt['y'])**2
        
        for i in range(1, len(self.tree)):
            tree_pt = {
                'x': self.tree[i].position['x'],
                'y': self.tree[i].position['y']
            }
            
            approx_dist = (sample_pt['x'] - tree_pt['x'])**2 + (sample_pt['y'] - tree_pt['y'])**2
            
            if approx_dist < min_dist:
                nearest_node = i
                min_dist = approx_dist
        
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
        nearest_pos = {
            'x': nearest_node.position['x'],
            'y': nearest_node.position['y']
        }        
        sample_pos = {
            'x': sampled_point[0],
            'y': sampled_point[1]
        }
        
        x_diff = sample_pos['x'] - nearest_pos['x']
        y_diff = sample_pos['y'] - nearest_pos['y']
        angle = math.atan2(y_diff, x_diff)
        
        x = nearest_pos['x'] * 0.05
        y = nearest_pos['y'] * 0.05
        
        new_pos = {
            'x': round((x + 0.5 * np.cos(angle)) / 0.05),
            'y': round((y + 0.5 * np.sin(angle)) / 0.05)
        }
        
        new_node = Vertex(position=new_pos, parent=nearest_node, input_dir=angle, cost=nearest_node.cost + abs(angle))
        
        return new_node

    def get_line(self, x0, y0, x1, y1):
        """
        Acquires all occupancy grid cells which intersect with the line segment
        connecting (x0, y0) to (x1, y1)
        
        Reference:
            https://forum.gamemaker.io/index.php?threads/how-to-find-every-square-a-line-passes-through.101130/
        
        Args:
            x0 (float): x coordinate in map frame for start point
            y0 (float): y coordinate in map frame for start point
            x1 (float): x coordinate in map frame for end point
            y1 (float): y coordinate in map frame for end point
        Returns:
            list of points of the form (x,y) for cell indices that are on the line
        """
        
        line = []
        cx0, cy0 = int(round(x0 / 0.05)), int(round(y0 / 0.05))
        cx1, cy1 = int(round(x1 / 0.05)), int(round(y1 / 0.05))
        m = float("inf") if cx0 == cx1 else (cy1 - cy0) / (cx1 - cx0)
        
        xdir = 1 if cx0 < cx1 else -1
        ydir = 1 if cy0 < cy1 else -1
        
        move_x = abs(cx1 - cx0) >= abs(cy1 - cy0)
        move_y = abs(cy1 - cy0) >= abs(cx1 - cx0)
        
        if move_x:
            line_eq = lambda x: m * (x - cx0) + cy0
            
            for x in range(cx0, cx1 + xdir, xdir):
                y         = line_eq(x)        # y value at x position on line
                y_cell    = round(y)          # y value of cell
                y_left    = line_eq(x - 0.5)  # y value at left edge of cell
                y_right   = line_eq(x + 0.5)  # y value at right edge of cell
                cell_down = y_cell - 1        # y value of cell below
                cell_up   = y_cell + 1        # y value of cell above
                
                line.append((x, y_cell))
                
                if abs(y_left - cell_down) <= 0.5 or abs(y_right - cell_down) <= 0.5:
                    line.append((x, cell_down))
                if abs(y_left - cell_up) <= 0.5 or abs(y_right - cell_up) <= 0.5:
                    line.append((x, cell_up))

        elif move_y:
            line_eq = lambda y: (y - cy0) / m + cx0
            
            for y in range(cy0, cy1 + ydir, ydir):
                x          = line_eq(y)        # x value at y position on line
                x_cell     = round(x)          # x value of cell
                x_down     = line_eq(y - 0.5)  # x value at bottom edge of cell
                x_up       = line_eq(y + 0.5)  # x value at top edge of cell
                cell_left  = x_cell - 1        # x value of left cell
                cell_right = x_cell + 1        # x value of right cell
                
                line.append((x_cell, y))
                
                if abs(x_down - cell_left) <= 0.5 or abs(x_up - cell_left) <= 0.5:
                    line.append((cell_left, y))
                if abs(x_down - cell_right) <= 0.5 or abs(x_up - cell_right) <= 0.5:
                    line.append((cell_right, y))
        
        return line

    def render_line(self, x0, y0, x1, y1):
        line = self.get_line(x0, y0, x1, y1)
    
        markers = MarkerArray()
        
        j = 0
        
        for pt in line:
            marker = Marker()
                    
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ns_linemarkers"

            marker.id = j
            marker.type = Marker.CUBE
            marker.action = Marker.MODIFY

            marker.pose.position.x = pt[0] * 0.05
            marker.pose.position.y = pt[1] * 0.05
            marker.pose.position.z = 1.0

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.04
            marker.scale.y = 0.04
            marker.scale.z = 0.04

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            
            markers.markers.append(marker)
            
            j += 1
        
        self.markers_pub.publish(markers)

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
        
        x0, y0 = nearest_node.position['x'] * 0.05, nearest_node.position['y'] * 0.05
        x1, y1 = new_node.position['x'] * 0.05, new_node.position['y'] * 0.05
        
        line = self.get_line(x0, y0, x1, y1)
        
        for pt in line:
            x = pt[0]
            y = pt[1]
            
            if f"({x},{y})" in self.local_occ_grid:
                return True
        
        return False

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
        return ((latest_added_node.position['x'] * 0.05 - goal_x)**2 + (latest_added_node.position['y'] * 0.05 - goal_y)**2)**0.5 <= 0.5

    def find_path(self, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        
        node = latest_added_node
        
        while node != None:
            path.insert(0, node)
            node = node.parent
        
        return path

    def render_path(self):
        markers = MarkerArray()
        
        j = 0
        
        for node in self.key_points:
            parent = node.parent
            
            marker = Marker() # actual marker for node
            
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ns_path"
            
            marker.id = j
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = node.position['x'] * 0.05
            marker.pose.position.y = node.position['y'] * 0.05
            marker.pose.position.z = 1.5
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            
            j += 1
            
            # create a line strip if the parent node exists
            if parent != None:
                line_strip = Marker()
                
                line_strip.header.frame_id = "map"
                line_strip.header.stamp = self.get_clock().now().to_msg()
                line_strip.ns = "ns_path"
                
                line_strip.id = j
                line_strip.type = Marker.LINE_STRIP
                line_strip.action = Marker.ADD
                
                line_strip.scale.x = 0.1
                
                line_strip.color.r = 1.0
                line_strip.color.g = 1.0
                line_strip.color.a = 0.8
                
                pt = Point(x=node.position['x'] * 0.05, y=node.position['y'] * 0.05, z=1.5)
                marker.points.append(pt)
                pt = Point(x=parent.position['x'] * 0.05, y=parent.position['y'] * 0.05, z=1.5)
                marker.points.append(pt)
            
            markers.markers.append(marker)
        
        self.markers_pub.publish(markers)

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