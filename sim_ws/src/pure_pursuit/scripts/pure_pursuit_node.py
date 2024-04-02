#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import csv
import numpy as np
import math
import atexit
from scipy.interpolate import interp1d, CubicSpline
from scipy.spatial.distance import euclidean

from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler, euler_from_quaternion

root_dir = "/home/vy/GalacticRacing/sim_ws/src"

def point_curavture(x, y, window_size):
    curvatures = np.array([0] * len(x))
    
    for i in range(len(x)):
        start_idx = max(0, i - window_size // 2)
        end_idx = min(len(x), i + window_size // 2 + 1)
        
        x_d1 = np.gradient(x[start_idx:end_idx])
        y_d1 = np.gradient(y[start_idx:end_idx])
        x_d2 = np.gradient(x_d1)
        y_d2 = np.gradient(y_d1)
        
        point_curavture = np.abs(x_d1 * y_d2 - y_d1 * x_d2) / ((x_d1**2 + y_d1**2)**1.5)
        
        curvatures[i] = np.mean(point_curavture)
    
    return curvatures

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Define subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped, "/pf/viz/inferred_pose", self.pose_callback, 10
        )

        # Define publishers
        self.markers_pub = self.create_publisher(
            MarkerArray, "/visualization_marker_array", 10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/drive", 10
        )

        self.range_pub = self.create_publisher(
            LaserScan, "/sample_range", 10
        )

        self.track_pub = self.create_publisher(
            LaserScan, "/track_scan", 10
        )

        self.sample_radius = 1.0
        self.markers = []
        self.path = []
        self.track_pt = (0.0, 0.0)

    def scan_callback(self, scan_msg):
        # render 360 degree circle around car to visualize sample radius
        new_scan : LaserScan = scan_msg
        new_scan.angle_min = math.radians(-180.0)
        new_scan.angle_max = math.radians(179.0)
        new_scan.angle_increment = math.radians(1.0)
        new_scan.ranges = [self.sample_radius] * 360

        self.range_pub.publish(new_scan)

        new_scan.ranges = [0.0] * 360
        new_scan.ranges[int(math.degrees(self.track_pt[1])) + 180] = self.track_pt[0]

        self.track_pub.publish(new_scan)

    def pose_callback(self, pose_msg):
        # find closest point to car
        min_pt = None
        min_dist = -1.0
        min_idx = 0

        for i in range(len(self.path)):
            point = self.path[i]

            dist = ((point['x'] - pose_msg.pose.position.x)**2 + (point['y'] - pose_msg.pose.position.y)**2)**0.5

            if min_dist < 0 or dist < min_dist:
                min_dist = dist
                min_pt = point
                min_idx = i
        
        # set sample radius based on closest point
        self.sample_radius = min_pt['lookahead']

        # find furthest point ahead of min_pt that's still within sample radius
        max_pt = min_pt
        max_dist = min_dist

        idx = (min_idx + 1) % len(self.path)

        while idx != min_idx:
            point = self.path[idx]

            dist = ((point['x'] - pose_msg.pose.position.x)**2 + (point['y'] - pose_msg.pose.position.y)**2)**0.5

            if dist < self.sample_radius and dist > max_dist:
                max_pt = point
                max_dist = dist
            elif dist > self.sample_radius:
                break

            idx = (idx + 1) % len(self.path)

        # acquire angular difference between car's orientation and orientation to point from car
        quaternion = (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)
        travel_angle = euler_from_quaternion(quaternion)[2]

        vc = (np.cos(travel_angle), np.sin(travel_angle))
        v3 = (max_pt['x'] - pose_msg.pose.position.x, max_pt['y'] - pose_msg.pose.position.y)

        mag_v3 = (v3[0]*v3[0] + v3[1]*v3[1])**0.5
        vc_cross_v3 = vc[0] * v3[1] - vc[1] * v3[0]
        vc_dot_v3 = vc[0] * v3[0] + vc[1] * v3[1]

        abs_angle = np.arccos(vc_dot_v3 / mag_v3)
        angle = (-1 if vc_cross_v3 < 0 else 1) * abs_angle

        if angle > math.radians(180.0):
            angle -= math.radians(360.0)
        elif angle < math.radians(-180.0):
            angle += math.radians(360.0)

        self.track_pt = (max_dist, angle)

        # clamp steering angle to valid range
        if angle < math.radians(-20.0):
            angle = math.radians(-20.0)
        elif angle > math.radians(20.0):
            angle = math.radians(20.0)

        # jesus take the wheel
        drive_msg = AckermannDriveStamped()
        # drive_msg.drive.speed = 0.0
        drive_msg.drive.speed = min_pt['speed']
        drive_msg.drive.steering_angle = angle

        self.drive_pub.publish(drive_msg)
        
    def renderPath(self):
        markers = MarkerArray()

        j = 0

        for waypoint in self.path:
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ns_interpmarkers"

            marker.id = j
            marker.type = 1
            marker.action = Marker.ADD

            marker.pose.position.x = waypoint["x"]
            marker.pose.position.y = waypoint["y"]
            marker.pose.position.z = 0.2

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            markers.markers.append(marker)

            j += 1

        self.markers_pub.publish(markers)

    def deletePath(self):
        self.path = []
        markers = MarkerArray()

        marker = Marker()
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)

        self.markers_pub.publish(markers)
    
    def loadPath(self):
        self.deletePath()

        filename = input("Path input filename? ")

        pathCSV = open(root_dir + "/pure_pursuit/paths/" + filename, "r")

        reader = csv.reader(pathCSV, delimiter=',')

        self.path = []

        for row in reader:
            self.path.append({
                "x": float(row[0]),
                "y": float(row[1]),
                "rot": float(row[2]),
                "speed": float(row[3]),
                "lookahead": float(row[4])
            })
            
        pathCSV.close()
    
    def halt(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0

        self.drive_pub.publish(drive_msg)


def main(args=None):

    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    atexit.register(pure_pursuit_node.halt)

    pure_pursuit_node.loadPath()
    pure_pursuit_node.renderPath()

    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
