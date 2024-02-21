#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import copy

from typing import (
    List, Tuple, Dict
)

float32 = np.float32

def deg_to_rad(deg : float32) -> float32:
    return (deg * np.pi) / 180.0

def rad_to_deg(rad : float32) -> float32:
    return (rad * 180.0) / np.pi

# refresh frequencies
SCAN_REFRESH = 10
DRIVE_REFRESH = 10

# scan preprocessing constants
RANGE_SIZE : int = 1080
WIND_SIZE  : int = 5
FOV : float = deg_to_rad(270.0)

# distance processing
DIST_THRESH  : float32 = 6.00 # meters
DISP_EPSILON : float32 = 0.20 # meters

# car dimensions
CAR_WIDTH : float32 = 0.45 # meters

def idx_to_rad(idx : int) -> float32:
    return idx * FOV / (RANGE_SIZE - 1) - FOV / 2.0

def rad_to_idx(rad : float32) -> int:
    return int(np.round((rad + FOV / 2.0) * (RANGE_SIZE / FOV)))

LEFT : int = rad_to_idx(deg_to_rad(90.0))
RIGHT : int = rad_to_idx(deg_to_rad(-90.0))

class ReactiveFollowGap(Node):
    """ 
    Implement Follow the Gap on the car
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        scan_mod_topic = '/mod_scan'

        # subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, lidarscan_topic, self.lidar_callback, SCAN_REFRESH
        )

        # publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, DRIVE_REFRESH
        )

        self.laser_pub = self.create_publisher(
            LaserScan, scan_mod_topic, SCAN_REFRESH
        )
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        for i in range(RIGHT, LEFT + 1):
            if ranges[i] > DIST_THRESH:
                ranges[i] = float("inf")
                # ranges[i] = DIST_THRESH
            elif ranges[i] == float("nan"):
                ranges[i] = 0.0
    
    def disparity_extender(self, ranges):
        disp_ranges = copy.deepcopy(ranges) # deep copy of ranges
        disp_idxs : List[Tuple[int, float32, float32]] = [] # list of disparity indices, distances, and differences
        radius : float32 = CAR_WIDTH / 2

        for i in range(RIGHT, LEFT):
            right_idx : int = i
            left_idx  : int = i + 1

            diff : float32 = ranges[left_idx] - ranges[right_idx]
            theta : float32 = 0.0 # rad

            if abs(diff) > DISP_EPSILON:
                if diff < 0:  # left side is closer than right side
                    dist : float32 = ranges[left_idx]

                    if abs(radius / dist) < np.pi/2:
                        theta = np.arctan(radius / dist)
                    
                    targ_idx : int = rad_to_idx(idx_to_rad(left_idx) - theta)

                    if targ_idx < RIGHT:
                        targ_idx = RIGHT

                    targ_offs : int = left_idx - targ_idx

                    disp_idxs.append((targ_idx, dist, diff))

                    for j in range(targ_offs):
                        if ranges[left_idx] < disp_ranges[right_idx - j]:
                            disp_ranges[right_idx - j] = ranges[left_idx]
                else:         # right side is closer than left side
                    dist : float32 = ranges[right_idx]

                    if abs(radius / dist) < np.pi/2:
                        theta = np.arctan(radius / dist)
                    
                    targ_idx : int = rad_to_idx(idx_to_rad(right_idx) + theta)

                    if targ_idx > LEFT:
                        targ_idx = LEFT

                    targ_offs : int = targ_idx - right_idx

                    disp_idxs.append((targ_idx, dist, diff))

                    for j in range(targ_offs):
                        if ranges[right_idx] < disp_ranges[left_idx + j]:
                            disp_ranges[left_idx + j] = ranges[right_idx]

        return disp_ranges
    
    def furthest_idx(self, ranges):
        Rmax_idx : int = RIGHT
        Lmax_idx : int = LEFT

        for i in range(RIGHT + 1, LEFT):
            if ranges[i] > ranges[Rmax_idx]:
                Rmax_idx = i
        
        for i in range(LEFT, RIGHT - 1, -1):
            if ranges[i] > ranges[Lmax_idx]:
                Lmax_idx = i

        print(f"Left: {Lmax_idx}, Right: {Rmax_idx}")
        return (Rmax_idx + Lmax_idx) // 2
    
    def bubble(self, ranges):
        close_idx : int = self.closest_idx(ranges)
        theta : float32 = 0.0

        if abs(CAR_WIDTH / (2 * ranges[close_idx])) < np.pi / 2:
            theta : float32 = 1.0 * np.arctan(CAR_WIDTH / (2 * ranges[close_idx]))

        right_idx : int = rad_to_idx(idx_to_rad(close_idx) - theta)
        left_idx  : int = rad_to_idx(idx_to_rad(close_idx) + theta)

        if ranges[close_idx] < 0.9:
            for i in range(right_idx, left_idx + 1):
                ranges[i] = ranges[close_idx]

    def closest_idx(self, ranges):
        min_idx : int = 540

        for i in range(RIGHT + 235, LEFT - 234):
            if ranges[i] < ranges[min_idx]:
                min_idx = i

        return min_idx

    def lidar_callback(self, data):
        """
        Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """

        # deep copy of laser scan message
        new_scan : LaserScan = copy.deepcopy(data)

        # preprocess scan ranges
        self.preprocess_lidar(new_scan.ranges)

        disp_ranges, disp_list = self.disparity_extender(new_scan.ranges)
        self.bubble(disp_ranges)

        new_scan.ranges = disp_ranges

        self.laser_pub.publish(new_scan)

        best_idx : int = self.furthest_idx(disp_ranges)

        # Determine steering angle from best point, if valid
        angle : float32

        if best_idx != -1 and disp_ranges[best_idx] > 3.0:
            angle = idx_to_rad(best_idx)
        else:
            angle = 0.0

        # Clamp angle between min and max steering angle
        if angle < deg_to_rad(-20.0):
            angle = deg_to_rad(-20.0)
        elif angle > deg_to_rad(20.0):
            angle = deg_to_rad(20.0)

        # Adjust velocity based on distance
        velocity : float32
        if data.ranges[540] < 0.65:
            velocity = 0.0
        elif data.ranges[540] > DIST_THRESH:
            velocity = 2.0
        elif data.ranges[540] < 2.0:
            velocity = 1.0
        else:
            velocity = 1.0

        #Publish Drive message
        msg = AckermannDriveStamped()
        # msg.drive.speed = 0.0
        msg.drive.speed = velocity
        msg.drive.steering_angle = angle
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    print("Gap Follow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
