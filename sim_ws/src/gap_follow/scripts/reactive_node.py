#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

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
DIST_THRESH  : float32 = 3.00 # meters
DISP_EPSILON : float32 = 0.60 # meters
CORN_EPSILON : float32 = 0.02 # meters

# car dimensions
CAR_WIDTH  : float32 = 0.45 # meters

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

        self.steering_angle = 0.0 # rads relative to forward (left: > 0.0, right: < 0.0)

        self.kp = 1.0
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        for i in range(RIGHT, LEFT + 1):
            if ranges[i] > DIST_THRESH:
                ranges[i] = DIST_THRESH # too far away; make it "infinity"
            elif ranges[i] == float("nan"):
                ranges[i] = 0.0

    def disparity_extender(self, ranges):
        disp_ranges = list(ranges)

        for i in range(RIGHT, LEFT):
            idx      : int = i      # query
            next_idx : int = i + 1  # left of query

            left_diff : float32 = ranges[idx] - ranges[next_idx]

            if abs(left_diff) > DISP_EPSILON:
                if left_diff < 0:  # object leaving scope
                    theta : float32 = 0.0

                    if abs(CAR_WIDTH / (2 * ranges[i])) < 1:
                        theta : float32 = np.arcsin(CAR_WIDTH / (2 * ranges[i]))

                    targ_idx : int = rad_to_idx(idx_to_rad(i) + theta)
                    targ_offs = targ_idx - i

                    for j in range(targ_offs):
                        if ranges[i + j] > ranges[i]:
                            disp_ranges[i + j] = ranges[i]
                else:                  # object entering scope
                    theta : float32 = 0.0

                    if abs(CAR_WIDTH / (2 * ranges[i + 1])) < 1:
                        theta : float32 = np.arcsin(CAR_WIDTH / (2 * ranges[i + 1]))

                    targ_idx : int = rad_to_idx(idx_to_rad(i + 1) - theta)
                    targ_offs = i + 1 - targ_idx

                    for j in range(targ_offs):
                        if disp_ranges[i - j] > disp_ranges[i + 1]:
                            disp_ranges[i - j] = ranges[i + 1]

        return disp_ranges

    def furthest_idx(self, ranges):
        max_right : int = RIGHT
        max_left  : int = LEFT

        for i in range(RIGHT + 1, LEFT + 1):
            if ranges[i] > ranges[max_right]:
                max_right = i
        
        for i in range(LEFT, RIGHT - 1, -1):
            if ranges[i] > ranges[max_left]:
                max_left = i
        
        return (max_left + max_right) // 2

    def lidar_callback(self, data):
        """
        Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = list(data.ranges)
        new_scan : LaserScan = data
        
        # preprocess range data
        self.preprocess_lidar(new_scan.ranges)

        # filter range data using disparity extender algorithm
        new_scan.ranges = self.disparity_extender(new_scan.ranges)

        # get best index by finding the largest gap and aiming towards its center
        # best_idx = self.max_gap(new_scan.ranges)

        self.laser_pub.publish(new_scan)

        # get best index by finding longest distance in range and aiming there
        best_idx = self.furthest_idx(new_scan.ranges)

        # Determine steering angle from best point, if valid
        angle : float32

        if best_idx != -1:
            angle = idx_to_rad(best_idx)
        else:
            angle = 0.0

        # Clamp angle between min and max steering angle
        if angle < deg_to_rad(-20.0):
            angle = deg_to_rad(-20.0)
        elif angle > deg_to_rad(20.0):
            angle = deg_to_rad(20.0)

        # print(f"Steering to angle: {rad_to_deg(angle)}")
        self.steering_angle = angle
        # self.steering_angle = 0.0

        # Adjust velocity based on distance
        velocity : float32
        if new_scan.ranges[540] < 0.75:
            velocity = 0.0
        else:
            velocity = ranges[540] * 0.8

        #Publish Drive message
        msg = AckermannDriveStamped()
        # msg.drive.speed = 0.5
        msg.drive.speed = velocity
        msg.drive.steering_angle = self.steering_angle
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
