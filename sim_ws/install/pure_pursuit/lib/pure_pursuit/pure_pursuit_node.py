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

        self.tracker_pub = self.create_publisher(
            Marker, "/path_tracker", 10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/drive", 10
        )

        self.range_pub = self.create_publisher(
            LaserScan, "/sample_range", 10
        )

        self.sample_radius = 1.0
        self.markers = []
        self.path = []

    def scan_callback(self, scan_msg):
        # render 360 degree circle around car to visualize sample radius
        new_scan : LaserScan = scan_msg
        new_scan.angle_min = math.radians(-180.0)
        new_scan.angle_max = math.radians(179.0)
        new_scan.angle_increment = math.radians(1.0)
        new_scan.ranges = [self.sample_radius] * 360

        self.range_pub.publish(new_scan)

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

        idx = min_idx + 1

        while idx != min_idx:
            point = self.path[idx]

            dist = ((point['x'] - pose_msg.pose.position.x)**2 + (point['y'] - pose_msg.pose.position.y)**2)**0.5

            if dist < self.sample_radius and dist > max_dist:
                max_pt = point
                max_dist = dist
            elif dist > self.sample_radius:
                break
                
            idx += 1
        
        # mark the target point
        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "ns_tracker"

        target_marker.id = idx
        target_marker.type = 0
        target_marker.action = 0

        target_marker.pose.position.x = max_pt['x']
        target_marker.pose.position.y = max_pt['y']
        target_marker.pose.position.z = 0.3

        target_marker.pose.orientation.x = 0.0
        target_marker.pose.orientation.y = 0.0
        target_marker.pose.orientation.z = 0.0
        target_marker.pose.orientation.w = 1.0

        target_marker.scale.x = 0.25
        target_marker.scale.y = 0.25
        target_marker.scale.z = 0.25

        target_marker.color.r = 0.0
        target_marker.color.g = 0.0
        target_marker.color.b = 1.0
        target_marker.color.a = 1.0

        self.tracker_pub.publish(target_marker)

        # transform target point into car's frame of reference
        quaternion = (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)
        travel_angle = euler_from_quaternion(quaternion)[2]

        trans_pt = {
            'x': max_pt['x'] - pose_msg.pose.position.x,
            'y': max_pt['y'] - pose_msg.pose.position.y
        }

        sin, cos = np.sin(travel_angle), np.cos(travel_angle)

        trans_pt = {
            'x': trans_pt['x'] * cos - trans_pt['y'] * sin + pose_msg.pose.position.x,
            'y': trans_pt['x'] * sin + trans_pt['y'] * cos + pose_msg.pose.position.y
        }

        # get steering angle to transformed point
        heading_error = np.arctan2(trans_pt['x'] - pose_msg.pose.position.x, trans_pt['y'] - pose_msg.pose.position.y) - travel_angle
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        angle = -np.arctan((0.648 * np.sin(heading_error)) / self.sample_radius)

        # clamp steering angle to valid range
        if angle < math.radians(-20.0):
            angle = math.radians(-20.0)
        elif angle > math.radians(20.0):
            angle = math.radians(20.)

        # jesus take the wheel
        drive_msg = AckermannDriveStamped()
        # drive_msg.drive.speed = 0.0
        drive_msg.drive.speed = max_pt['speed']
        drive_msg.drive.steering_angle = angle

        self.drive_pub.publish(drive_msg)
    
    def get_path(self, filename):
        # open waypoint file
        waypoint_csv = open(root_dir + '/pure_pursuit/paths/' + filename + '.csv', 'r')

        reader = csv.reader(waypoint_csv, delimiter=',')
        markers = MarkerArray()

        for idx, row in enumerate(reader):
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ns_markers"

            marker.id = idx
            marker.type = 0
            marker.action = 0

            marker.pose.position.x = float(row[0])
            marker.pose.position.y = float(row[1])
            marker.pose.position.z = 0.3

            quaternion = quaternion_from_euler(0.0, 0.0, float(row[2]))

            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]

            marker.scale.x = 0.25
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.markers.append(marker)

            self.path.append({
                'x': float(row[0]),
                'y': float(row[1]),
                'theta': float(row[2]),
                'speed': 0.5,
                'lookahead': 1.0
            })

        markers.markers = self.markers

        self.markers_pub.publish(markers)
    
    def interp_path(self):

        x = np.array([value['x'] for value in self.path])
        y = np.array([value['y'] for value in self.path])

        num_pts = len(self.path) - 1

        cumulative_sum = 0.0
        cumulative_dists = [0.0] * num_pts

        for i in range(num_pts):
            cumulative_sum += ((x[(i+1) % num_pts] - x[i])**2 + (y[(i+1) % num_pts] - y[i])**2)**0.5
            cumulative_dists[i] = cumulative_sum

        cumulative_dists = [0.0] + cumulative_dists

        norm_dists = [value / cumulative_dists[-1] for value in cumulative_dists]

        x_interp = CubicSpline(norm_dists, x, bc_type="natural")
        y_interp = CubicSpline(norm_dists, y, bc_type="natural")

        t = np.linspace(0, 1, 300)

        self.path = []
        markers = MarkerArray()

        j = 0

        for _, i in enumerate(t):
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ns_interpmarkers"

            marker.id = j
            marker.type = 1
            marker.action = 0

            marker.pose.position.x = float(x_interp(i))
            marker.pose.position.y = float(y_interp(i))
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

            self.path.append({
                'x': float(x_interp(i)),
                'y': float(y_interp(i)),
                'theta': 0.0, # placeholder; compute angle to next path point
                'speed': 0.5, # placeholder; compute best speed at each point
                'lookahead': 1.0 # placeholder; dynamic lookahead for each node in the path to account for turns/straight track sections
            })

            markers.markers.append(marker)

            j += 1

        self.markers_pub.publish(markers)
    
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

    pure_pursuit_node.get_path('levine_blocked')
    pure_pursuit_node.interp_path()

    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
