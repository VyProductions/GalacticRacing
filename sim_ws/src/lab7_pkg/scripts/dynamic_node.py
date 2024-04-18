#!/usr/bin/env python3

# ==================================================================================================
# Standard Imports
# ==================================================================================================

import csv
import numpy as np
import math
import atexit
import os
from scanf import scanf
from PIL import Image # for png map processing

# ==================================================================================================
# ROS Imports
# ==================================================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion

# ==================================================================================================
# Utility Imports
# ==================================================================================================

import lab7_pkg.helper as hp
import lab7_pkg.variables as vars

# ==================================================================================================
# Miscellaneous Imports
# ==================================================================================================

from typing import *

# ==================================================================================================
# Vec2 Class
# ==================================================================================================

class Vec2:
    def __init__(
        self,
        x : int = 0,
        y : int = 0
    ):
        self.x = int(x)
        self.y = int(y)

    # Generated by ChatGPT for use as a key in a dictionary
    def __eq__(self, other):
        if isinstance(other, Vec2):
            return self.x == other.x and self.y == other.y
        return False

    def __hash__(self):
        return hash((self.x, self.y))

    def __repr__(self):
        return f"({self.x},{self.y})"

class Vec2f:
    def __init__(
        self,
        x : float = 0.0,
        y : float = 0.0
    ):
        self.x = float(x)
        self.y = float(y)

# ==================================================================================================
# Vertex Class
# ==================================================================================================

class Vertex:
    def __init__(
        self,
        position : Vec2 = Vec2(),
        parent = None,
        input_dir : float = 0.0,
        cost : float = 0.0
    ):
        self.position = position
        self.parent = parent
        self.input_dir = input_dir
        self.cost = cost
    
    def __repr__(self):
        return f"{self.position} : {self.cost} <- ({'No Parent' if self.parent == None else 'Recursive Parent' if self.parent == self else self.parent})"

# ==================================================================================================
# Dynamic Node Class
# ==================================================================================================

class Dynamic(Node):
    def __init__(self):
        super().__init__('dynamic_node')
        
        # ==========================================================================================
        # Subscriber / Publisher Topics
        # ==========================================================================================
        
        # Car
        scan_topic  = '/scan'
        pose_topic  = '/pf/viz/inferred_pos'
        odom_topic  = '/ego_racecar/odom'
        drive_topic = '/drive'
        
        # Map
        global_occ_topic = '/global_occupancy'
        local_occ_topic  = '/local_occupancy'
        
        # Pathing
        sample_grid_topic = '/sample_grid'
        path_topic        = '/path_view'
        goals_topic       = '/goal_view'
        
        # ==========================================================================================
        # Subscribers / Publishers
        # ==========================================================================================
        
        # Car
        self.scan_sub  = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.pose_sub  = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
        self.odom_sub  = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        
        # Map
        self.global_occ_pub = self.create_publisher(OccupancyGrid, global_occ_topic, 10)
        self.local_occ_pub  = self.create_publisher(OccupancyGrid, local_occ_topic, 10)

        # Pathing
        self.sample_grid_pub = self.create_publisher(MarkerArray, sample_grid_topic, 10)
        self.path_pub        = self.create_publisher(MarkerArray, path_topic, 10)
        self.goals_pub       = self.create_publisher(MarkerArray, goals_topic, 10)
        
        # ==========================================================================================
        # Runtime Variables
        # ==========================================================================================
        
        # Car
        self.position : Vec2f = None  # Physical position of car on map
        self.cell_pos : Vec2  = None  # Cell position of car on map
        self.rotation : float = None  # Rotation relative to map frame (radians)
        
        # Map
        self.global_occ_grid : Dict[Vec2, float] = {}  # Static global occupancy grid
        self.free_space_grid : Dict[Vec2, float] = {}  # Static free space grid for track
        self.local_occ_grid  : Dict[Vec2, float] = {}  # Dynamic local occupancy grid
        self.map_config      : Dict[str, Any]    = {}  # Map configuration details
        
        # Pathing
        self.curr_goal  : Vertex       = Vertex()      # Current goal as a node
        self.known_free : Vec2         = Vec2(0, 100)  # Cell known to be in free space
        self.nodes      : List[Vertex] = []            # List of nodes for I-RRT*
        
        # ==========================================================================================
        # Runtime Constants
        # ==========================================================================================
        
        # Occupancy Grid
        self.cell_size   : float = 0.05
        self.local_width : int   = 40
        self.local_depth : int   = 40
        self.bubble_offs : List[Vec2] = [
            Vec2( 0,  0), Vec2( 0, -1), Vec2( 1,  0), Vec2(-1,  0), Vec2( 0,  1), Vec2( 1,  1),
            Vec2( 1, -1), Vec2(-1,  1), Vec2(-1, -1), Vec2( 0, -2), Vec2( 2,  0), Vec2(-2,  0),
            Vec2( 0,  2), Vec2( 2, -1), Vec2(-1,  2), Vec2(-1, -2), Vec2(-2,  1), Vec2(-2, -1),
            Vec2( 1, -2), Vec2( 1,  2), Vec2( 2,  1), Vec2( 2, -2), Vec2( 2,  2), Vec2(-2,  2),
            Vec2(-2, -2), Vec2( 0, -3), Vec2( 0,  3), Vec2( 3,  0), Vec2(-3,  0), Vec2(-1,  3),
            Vec2(-1, -3), Vec2( 3, -1), Vec2( 1, -3), Vec2( 3,  1), Vec2(-3,  1), Vec2( 1,  3),
            Vec2(-3, -1), Vec2( 3, -2), Vec2( 2,  3), Vec2(-3, -2), Vec2( 3,  2), Vec2( 2, -3),
            Vec2(-2,  3), Vec2(-2, -3), Vec2(-3,  2), Vec2( 0,  4), Vec2( 0, -4), Vec2(-4,  0),
            Vec2( 4,  0)
        ]
        
        # Environment Information
        self.root_dir : str = self.get_root_dir()  # Parent directory to package folders
        
        # File Information
        self.global_occ_filename : str = self.get_global_occ_filename()  # Global occupancy filename
        self.goals_filename      : str = self.get_goals_filename()       # Local occupancy filename
        self.map_filename        : str = self.get_map_filename()         # Map information filename
        
        # Status
        self.mapping : bool = self.get_state()  # Whether the program is in mapping or driving mode
        
        # Pathing
        self.goal_list : List[Vec2] = self.read_goals()  # Ordered list of goal cells around the map
    
    # ==============================================================================================
    # Setup / Teardown Functions
    # ==============================================================================================
    
    def get_root_dir(self) -> str:
        """
        Query user for full path of parent folder to package folders
        
        Returns:
            Path string for root directory
        """
        
        while True:
            path : str = input("Enter full path to 'src' (or equivalent) directory: ")
            # path : str = '/home/vy/GalacticRacing/sim_ws/src'
            
            if os.path.exists(path):
                return path
            else:
                print("Invalid path. Try again.")
    
    def get_global_occ_filename(self) -> str:
        """
        Query user for global occupancy grid filename
        
        Returns:
            Full path to global occupancy grid file
        """
        
        while True:
            filename : str = input("Enter global occupancy filename: ")
            # filename : str = 'AEB_flexito.csv'
            
            if os.path.exists(self.root_dir + '/lab7_pkg/occ_grids/' + filename):
                return self.root_dir + '/lab7_pkg/occ_grids/' + filename
            else:
                print("File does not exist. Try again.")
    
    def get_goals_filename(self) -> str:
        """
        Query user for goal markers filename
        
        Returns:
            Full path to goal markers file
        """
        
        while True:
            filename : str = input("Enter goal marker filename: ")
            # filename : str = 'AEB_flexito.csv'
            
            if os.path.exists(self.root_dir + '/lab7_pkg/goals/' + filename):
                return self.root_dir + '/lab7_pkg/goals/' + filename
            else:
                print("File does not exist. Try again.")
    
    def get_map_filename(self) -> str:
        """
        Query user for map information filename
        
        Returns:
            Full path to map file (no extension)
        """
        
        while True:
            filename : str = input("Enter map filename (no extension): ")
            # filename : str = 'AEB_flexito'
            
            if (
                os.path.exists(self.root_dir + '/particle_filter/maps/' + filename + '.yaml') and
                (
                    os.path.exists(self.root_dir + '/particle_filter/maps/' + filename + '.png') or
                    os.path.exists(self.root_dir + '/particle_filter/maps/' + filename + '.pgm')
                )
            ):
                return self.root_dir + '/particle_filter/maps/' + filename
            else:
                print("Map image and config file do not exist. Try again.")
    
    def get_map_config(self) -> None:
        """
        Read configuration file to acquire dimensions and origin of map
        
        Returns:
            None
        """

        try:
            lines : List[str] = []
            
            for i in open(self.map_filename + '.yaml', "r"):
                lines.append(i)
            
            self.map_config["image"] = scanf("image: %s\n", lines[0])[0]
            self.map_config["mode"] = scanf("mode: %s\n", lines[1])[0]
            self.map_config["resolution"] = scanf("resolution: %f\n", lines[2])[0]
            originPos = scanf("origin: [%f, %f, %f]\n", lines[3])
            self.map_config["origin"] = Vec2f(x=originPos[0], y=originPos[1])
            self.map_config["negate"] = scanf("negate: %d\n", lines[4])[0]
            self.map_config["occupied_thresh"] = scanf("occupied_thresh: %f\n", lines[5])[0]
            self.map_config["free_thresh"] = scanf("free_thresh: %f", lines[6])[0]
            
            if self.map_config["image"].split('.')[-1] == "pgm":
                image_dims : str = ""
                
                with open(self.root_dir + '/particle_filter/maps/' + self.map_config["image"], "rb") as f:
                    first_line = f.readline().decode()
                    
                    if first_line != "P5\n":
                        print("Not a P5 image file.")
                    else:
                        # acquire image dimensions
                        image_dims = f.readline().decode()
                
                self.map_config["width"], self.map_config["height"] = scanf("%d %d\n", image_dims)
            elif self.map_config["image"].split('.')[-1] == "png":
                img = Image.open(self.root_dir + '/particle_filter/maps/' + self.map_config["image"], "r")
                
                self.map_config["width"], self.map_config["height"] = img.width, img.height
            else:
                print("Invalid image extension (.png, .pgm accepted).")
        except FileNotFoundError:
            print("Could not open map config file. Try again.")
        except Exception as e:
            print(f"Exception occurred: [{type(e)}] {e}")
    
    def get_state(self) -> bool:
        """
        Query user for program state (mapping / driving) to use during execution.
        
        Returns:
            True if the mapping state is active, otherwise False
        """
        
        while True:
            try:
                response = input('Which Mode ([M]apping, [D]riving): ')

                if response.find('M') == 0:
                    # Acquire map config information
                    self.get_map_config()
                    
                    return True
                elif response.find('D') == 0:
                    # Acquire saved global occupancy grid
                    self.global_occ_grid = self.read_global_occ()
                    
                    # Acquire map config information
                    self.get_map_config()
                    
                    # Flood-fill from where car is (assumed to be on track) to acquire free space
                    self.flood_fill()
                    
                    # Render global occupancy grid
                    self.render_global_occ()
                    
                    return False
            except Exception as e:
                print(f"Exception occurred: [{type(e)}] {e}")
            
            print("Invalid choice. Try again.")
    
    def halt(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        
        self.drive_pub.publish(drive_msg)
        
        if self.mapping:
            self.save_global_occ()
    
    # ==============================================================================================
    # File Interface Functions
    # ==============================================================================================
    
    def read_goals(self) -> List[Vec2f]:
        """
        Acquire directional list of goal positions from file
        
        Returns:
            List of goal positions in order
        """
        
        goal_list : List[Vec2f] = []
        
        try:
            goalsCSV = open(self.goals_filename, "r")
            
            reader = csv.reader(goalsCSV, delimiter=',')
            
            for row in reader:
                try:
                    goal_list.append(Vec2f(float(row[0]), float(row[1])))
                except ValueError:
                    print("Failed to convert row data into Vec2f object.")
                    print(f"  Content: '{row}'")
            
            goalsCSV.close()
        except FileNotFoundError:
            print("Could not open goals file.")
        
        return goal_list
        
    
    def read_global_occ(self) -> Dict[Vec2, float]:
        """
        Acquire global occupancy grid from file
        
        Returns:
            Map from a cell's position to its occupancy status
        """
        
        global_occ_grid : Dict[Vec2, float] = {}
        
        try:
            globalCSV = open(self.global_occ_filename, "r")
            
            reader = csv.reader(globalCSV, delimiter=',')
            
            for row in reader:
                try:
                    global_occ_grid[Vec2(int(row[0]), int(row[1]))] = float(row[2])
                except ValueError:
                    print("Failed to convert row data into [Vec2, float] mapping.")
                    print(f"  Content: '{row}'")
                except Exception as e:
                    print(f"Exception occurred: [{type(e)}] {e}")
            
            globalCSV.close()
        except FileNotFoundError:
            print("Could not open global occupancy grid file.")
        
        return global_occ_grid
    
    def save_global_occ(self) -> None:
        """
        Save global occupancy grid to file
        
        Returns:
            None
        """
        
        try:
            globalCSV = open(self.global_occ_filename, "w")
            
            writer = csv.writer(globalCSV, delimiter=',')
            
            for position, occ_status in self.global_occ_grid.items():
                writer.writerow([position.x, position.y, occ_status])
            
            globalCSV.close()
        except FileNotFoundError:
            print("Could not open global occupancy grid file.")
    
    # ==============================================================================================
    # Subscription Callback Functions
    # ==============================================================================================
    
    def scan_callback(self, scan_msg : LaserScan) -> None:
        """
        Create the global occupancy grid while mapping mode is selected, otherwise create the local
        occupancy grid while driving mode is selected
        
        Args:
            scan_msg (LaserScan): incoming scan message from subscribed topic
        Returns:
            None
        """
        
        # reset local occupcancy grid in driving mode
        if not self.mapping:
            self.local_occ_grid.clear()
        
        ranges = scan_msg.ranges
        
        if self.position != None:
            for i in range(vars.RANGE_SIZE):
                dist    = ranges[i] + 0.01  # extend point into occlusion for better grid cell accuracy
                car_dir = self.rotation
                theta   = car_dir + hp.idx_to_rad(i, vars.FOV, vars.RANGE_SIZE)
                
                x_las, y_las = 0.261 * np.cos(car_dir), 0.261 * np.sin(car_dir)
                x_offs, y_offs = x_las + dist * np.cos(theta), y_las + dist * np.sin(theta)
                
                occluded_cell = Vec2(
                    x = round((self.position.x + x_offs) / 0.05),
                    y = round((self.position.y + y_offs) / 0.05)
                )
                
                if self.mapping:
                    self.bubble(occluded_cell, self.global_occ_grid)
                else:
                    if self.local_cell(occluded_cell):
                        self.bubble(occluded_cell, self.local_occ_grid)

            self.curr_goal = Vertex(
                position=Vec2(
                    x=round(-1.8/0.05),
                    y=round(3.1/0.05)
                )
            )
            
            print("I-RRT* Starting")

            self.irrt_star(
                start=Vertex(position=self.cell_pos),
                goal=self.curr_goal,
                max_iter=1000,
                max_distance=10.0,
                goal_sample_rate=0.1
            )
            
            print("Rendering Path")
            
            self.render_path()
            
            print("Rendering Local Occupancy Grid")
            
            self.render_local_occ()
    
    def pose_callback(self, pose_msg : PoseStamped) -> None:
        """
        Update position and rotation of car using inferred pose
        
        Args:
            pose_msg (PoseStamped): incoming pose message from subscribed topic
        Returns:
            None
        """
        
        p : Point = pose_msg.pose.position
        q : Quaternion = pose_msg.pose.orientation
        
        self.position = Vec2f(p.x, p.y)
        self.cell_pos = Vec2(round(p.x / 0.05), round(p.y / 0.05))
        self.rotation = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    
    def odom_callback(self, odom_msg : Odometry) -> None:
        """
        Update position and rotation of car using simulated odometry
        
        Args:
            odom_msg (Odometry): incoming odometry message from subscribed topic
        Returns:
            None
        """
        
        p : Point = odom_msg.pose.pose.position
        q : Quaternion = odom_msg.pose.pose.orientation
        
        self.position = Vec2f(p.x, p.y)
        self.cell_pos = Vec2(round(p.x / 0.05), round(p.y / 0.05))
        self.rotation = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    
    # ==============================================================================================
    # Visualization Functions
    # ==============================================================================================
    
    def float_to_cell_coordinate(self, x, y):
        # Calculate scaled coordinates in terms of cell units
        x_scaled = x / self.cell_size
        y_scaled = y / self.cell_size

        # Calculate row index
        row = self.map_config["height"] - 1 - int(y_scaled * (self.map_config["height"] - 1))

        # Calculate column index
        col = int(x_scaled * (self.map_config["width"] - 1))

        return row, col
    
    def render_global_occ(self) -> None:
        """
        Create and publish an OccupancyGrid message containing the global occupancy grid to the
        rviz2 client
        
        Returns:
            None
        """
        
        global_occ : OccupancyGrid = OccupancyGrid()
        
        global_occ.header.frame_id = "map"
        global_occ.header.stamp = self.get_clock().now().to_msg()
        
        global_occ.info.map_load_time = self.get_clock().now().to_msg()
        global_occ.info.resolution = self.map_config["resolution"]
        global_occ.info.width = self.map_config["width"]
        global_occ.info.height = self.map_config["height"]
        global_occ.info.origin = Pose(
            position=Point(
                x=self.map_config["origin"].x,
                y=self.map_config["origin"].y,
                z=0.0
            ),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        
        num_cells = self.map_config["height"] * self.map_config["width"]
        
        global_occ.data = [0] * num_cells
        
        origin_pt : Vec2f = self.map_config["origin"]
        
        C, R = self.map_config["width"], self.map_config["height"]
        
        for r in range(R):
            for c in range(C):
                pos : Vec2 = Vec2(
                    x=c + int(origin_pt.x / self.cell_size),
                    y=r + int(origin_pt.y / self.cell_size)
                )
                
                if pos in self.global_occ_grid:
                    global_occ.data[r * C + c] = 100
                elif pos in self.free_space_grid:
                    global_occ.data[r * C + c] = 0
                else:
                    global_occ.data[r * C + c] = -1
        
        self.global_occ_pub.publish(global_occ)
    
    def render_local_occ(self) -> None:
        """
        Create and publish an OccupancyGrid message containing the local occupancy grid to the
        rviz2 client
        
        Returns:
            None
        """
        
        local_occ : OccupancyGrid = OccupancyGrid()
        
        local_occ.header.frame_id = "map"
        local_occ.header.stamp = self.get_clock().now().to_msg()
        
        local_occ.info.map_load_time = self.get_clock().now().to_msg()
        local_occ.info.resolution = self.map_config["resolution"]
        local_occ.info.width = self.map_config["width"]
        local_occ.info.height = self.map_config["height"]
        local_occ.info.origin = Pose(
            position=Point(
                x=self.map_config["origin"].x,
                y=self.map_config["origin"].y,
                z=0.0
            ),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        
        num_cells = self.map_config["height"] * self.map_config["width"]
        
        local_occ.data = [0] * num_cells
        
        origin_pt : Vec2f = self.map_config["origin"]
        car_pt    : Vec2f = self.position
        
        C, R = self.map_config["width"], self.map_config["height"]
        
        origin_cell_in_image : Vec2 = Vec2(
            x=-int(origin_pt.x / self.cell_size),
            y=-int(origin_pt.y / self.cell_size)
        )
        
        car_cell_in_image : Vec2 = Vec2(
            x=self.cell_pos.x + origin_cell_in_image.x,
            y=self.cell_pos.y + origin_cell_in_image.y
        )
        
        for r in range(-self.local_width, self.local_width + 1):
            for c in range(-self.local_depth, self.local_depth + 1):
                img_pos : Vec2 = Vec2(
                    x=c + car_cell_in_image.x + round(self.local_depth * np.cos(self.rotation)),
                    y=r + car_cell_in_image.y + round(self.local_width * np.sin(self.rotation))
                )
                
                pos : Vec2 = Vec2(
                    x=img_pos.x + int(origin_pt.x / 0.05),
                    y=img_pos.y + int(origin_pt.y / 0.05)
                )
                
                if (
                    img_pos.x >= 0 and img_pos.x < C and
                    img_pos.y >= 0 and img_pos.y < R
                ):
                    if pos in self.local_occ_grid:
                        local_occ.data[img_pos.y * C + img_pos.x] = 100
                    elif pos in self.free_space_grid:
                        local_occ.data[img_pos.y * C + img_pos.x] = 0
                    else:
                        local_occ.data[img_pos.y * C + img_pos.x] = -1
        
        self.local_occ_pub.publish(local_occ)
    
    def render_sample_grid(self) -> None:
        """
        Create and publish a MarkerArray message defining the local sample space around the car to
        the rviz2 client
        
        Returns:
            None
        """
    
    def render_path(self) -> None:
        """
        Create and publish a MarkerArray message defining the discovered path to the current goal to
        the rviz2 client
        
        Returns:
            None
        """
        
        print("  Clearing Render.")
        
        markers : MarkerArray = MarkerArray()
        
        marker : Marker = Marker()
        
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ns_rrt_path"
        
        marker.id = 0
        marker.action = Marker.DELETEALL
        
        markers.markers.append(marker)
        
        self.path_pub.publish(markers)
        
        markers.markers.clear()
        
        j = 1
        
        print("  Iterating Over Nodes.")
        
        for node in self.reconstruct_path(self.curr_goal):
            print("  Node:", node.position)
            
            marker : Marker = Marker()
            
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ns_rrt_path"
            
            marker.id = j
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = node.position.x * 0.05
            marker.pose.position.y = node.position.y * 0.05
            marker.pose.position.z = 1.0
            
            marker.scale.x = 0.04
            marker.scale.y = 0.04
            marker.scale.z = 0.04
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            
            markers.markers.append(marker)
            
            j += 1
        
        if len(markers.markers) > 0:
            self.path_pub.publish(markers)
        
        print("  Done.")
    
    def render_goals(self) -> None:
        """
        Create and publish a MarkerArray message defining the goals around the map to the rviz2
        client
        
        Returns:
            None
        """
    
    # ==============================================================================================
    # Occupancy Grid Functions
    # ==============================================================================================
    
    def bubble(self, position : Vec2, occ_grid : Dict[Vec2, float]) -> None:
        """
        Expand the occupation grid around the given position to account for the car's dimensions
        
        Args:
            position (Vec2): cell position in the occupancy grid to bubble around
            occ_grid (Dict[Vec2, float]): the occupancy grid
        Returns:
            None
        """
        
        for offs in self.bubble_offs:
            occ_grid[Vec2(position.x + offs.x, position.y + offs.y)] = 1.0
    
    def local_cell(self, position : Vec2) -> bool:
        """
        Determines if a specified cell is within the bounds of the local occupancy grid
        
        Args:
            position (Vec2): position of the cell to check
        Returns:
            True if the cell position is part of the local occupancy grid, False otherwise
        """
        
        p1 : Vec2  = Vec2(
            x=self.cell_pos.x + round(self.local_depth * np.cos(self.rotation)),
            y=self.cell_pos.y + round(self.local_width * np.sin(self.rotation))
        )
        
        cos = np.cos(self.rotation)
        sin = np.sin(self.rotation)
        x_diff = position.x - p1.x
        y_diff = position.y - p1.y
        
        transPoint : Vec2 = Vec2(
            x = round(x_diff * cos + y_diff * sin),
            y = round(y_diff * cos - x_diff * sin)
        )
        
        return (
            abs(transPoint.x) <= self.local_depth and
            abs(transPoint.y) <= self.local_width
        )
    
    def flood_fill(self):
        free_cells : Dict[Vec2, bool] = {
            self.known_free: True
        }
        
        proc_cells : Dict[Vec2, bool] = {}
        
        offs : List[Vec2] = [
            Vec2(-1, -1), Vec2( 0, -1), Vec2( 1, -1),
            Vec2(-1,  0),               Vec2( 1,  0),
            Vec2(-1,  1), Vec2( 0,  1), Vec2( 1,  1)
        ]
        
        while len(free_cells) > 0:
            curr_cell, _ = free_cells.popitem()
            
            proc_cells[curr_cell] = True
            
            self.free_space_grid[curr_cell] = 0.0
            
            for v in offs:
                new_pos : Vec2 = Vec2(x=curr_cell.x + v.x, y=curr_cell.y + v.y)
                
                if new_pos not in self.global_occ_grid and new_pos not in proc_cells:
                    free_cells[new_pos] = True
    
    # ==============================================================================================
    # Path Finding Functions
    # ==============================================================================================
    
    def distance(self, v1 : Vec2, v2 : Vec2) -> float:
        return np.sqrt((v1.position.x - v2.position.x)**2 + (v1.position.y - v2.position.y)**2)
    
    def generate_random_node(self, goal : Vertex, ellipse_center : Vec2, ellipse_axes : List[int], goal_sample_rate : float) -> Vertex:
        if np.random.rand() < goal_sample_rate:
            return goal
        else:
            while True:
                x = np.random.randint(0, self.map_config["width"])
                y = np.random.randint(0, self.map_config["height"])
                
                if ((x - ellipse_center.x) / ellipse_axes[0])**2 + ((y - ellipse_center.y) / ellipse_axes[1])**2 <= 1:
                    return Vertex(
                        position=Vec2(x, y)
                    )
    def nearest_node(self, random_node : Vertex):
        nearest = self.nodes[0]
        min_dist = self.distance(self.nodes[0], random_node)
        
        for node in self.nodes[1:]:
            dist = self.distance(node, random_node)
            
            if dist < min_dist:
                nearest = node
                min_dist = dist
        
        return nearest
    
    def steer(self, from_node, to_node, max_distance):
        if self.distance(from_node, to_node) < max_distance:
            return to_node
        else:
            theta = np.arctan2(to_node.position.y - from_node.position.y, to_node.position.x - from_node.position.x)
            return Vertex(
                position=Vec2(round(from_node.position.x + max_distance * np.cos(theta)), round(from_node.position.y + max_distance * np.sin(theta)))
            )
    
    def get_line(self, x0, y0, x1, y1):
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
    
    def is_collision_free(self, from_node : Vertex, to_node : Vertex):
        x0, y0 = from_node.position.x * 0.05, from_node.position.y * 0.05
        x1, y1 = to_node.position.x * 0.05, to_node.position.y * 0.05
        
        line = [(from_node.position.x, from_node.position.y)] \
            if x0 == x1 and y0 == y1 \
            else self.get_line(x0, y0, x1, y1)
        
        for pt in line:
            pos : Vec2 = Vec2(
                x=pt[0],
                y=pt[1]
            )
            
            if pos in self.global_occ_grid or pos in self.local_occ_grid:
                return False
        
        return True

    def rewire(self, new_node, max_distance):
        for node in self.nodes:
            if node == new_node.parent:
                continue
            if self.distance(node, new_node) > max_distance:
                continue
            if node.cost > new_node.cost + self.distance(node, new_node) and self.is_collision_free(node, new_node):
                node.parent = new_node
                node.cost = new_node.cost + self.distance(node, new_node)
    
    def irrt_star(self, start : Vertex, goal : Vertex, max_iter : int, max_distance : float, goal_sample_rate : float) -> None:
        self.nodes = [start]
        
        ellipse_axes = [
            max(self.map_config["width"], self.map_config["height"]),
            max(self.map_config["width"], self.map_config["height"])
        ]
        
        for _ in range(max_iter):
            ellipse_center : Vec2 = Vec2(goal.position.x, goal.position.y)
            random_node    : Vertex = self.generate_random_node(goal, ellipse_center, ellipse_axes, goal_sample_rate)
            nearest        : Vertex = self.nearest_node(random_node)
            new_node       : Vertex = self.steer(nearest, random_node, max_distance)
            
            if self.is_collision_free(nearest, new_node):
                new_node.parent = nearest
                new_node.cost = nearest.cost + self.distance(nearest, new_node)
                self.nodes.append(new_node)
                
                self.rewire(new_node, max_distance)
                
                if self.distance(new_node, goal) < max_distance:
                    goal.parent = new_node
                    goal.cost = new_node.cost + self.distance(new_node, goal)
                    
                    self.rewire(goal, max_distance)
                    
                    # Update ellipse size
                    # progress = len(self.nodes) / max_iter
                    # ellipse_axes = [
                    #     max(self.map_config["width"], self.map_config["height"]) * np.sqrt(1 - progress),
                    #     max(self.map_config["width"], self.map_config["height"]) * np.sqrt(1 - progress)
                    # ]
                    
                    return
    
    def reconstruct_path(self, goal_node : Vertex):
        path = []
        current = goal_node
        
        print("  Reconstructing Path.")
        print("   ", "No Goal" if current == None else current.parent)
        
        while current != None and current.parent != current:
            path.insert(0, current)
            current = current.parent
        
        print("  Done Reconstructing.")
        
        return path

def main(args=None):
    rclpy.init(args=args)
    
    print("Dynamic Node Initialized")
    
    dynamic_node = Dynamic()
    
    atexit.register(dynamic_node.halt)
    
    rclpy.spin(dynamic_node)
    
    dynamic_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()