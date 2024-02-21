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

import gap_follow.helper as hp
import gap_follow.variables as vars

float32 = np.float32

def idx_to_rad(idx : int) -> float32:
    return hp.idx_to_rad(idx, vars.FOV, vars.RANGE_SIZE)

def rad_to_idx(rad : float32) -> float32:
    return hp.rad_to_idx(rad, vars.FOV, vars.RANGE_SIZE)

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
            LaserScan, lidarscan_topic, self.lidar_callback, vars.SCAN_REFRESH
        )

        # publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, vars.DRIVE_REFRESH
        )

        self.laser_pub = self.create_publisher(
            LaserScan, scan_mod_topic, vars.SCAN_REFRESH
        )
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        sample_range : int = 10

        for i in range(vars.RANGE_SIZE):
            if ranges[i] > vars.DIST_THRESH:
                ranges[i] = vars.DIST_THRESH
            elif ranges[i] == float("nan"):
                ranges[i] = 0.0
            # force occlusion for gaps that are quite small
            elif i > sample_range and i < vars.RANGE_SIZE - sample_range and \
                ranges[i] > ranges[i-sample_range] + vars.DISP_EPSILON and ranges[i] > ranges[i+sample_range] + vars.DISP_EPSILON:
                ranges[i] = ranges[i-sample_range]

        tmp_ranges = copy.deepcopy(ranges)

        for i in range(vars.RIGHT, vars.LEFT + 1):
            sum : float32 = 0.0

            for j in range(-2, 3):
                sum += tmp_ranges[i + j]

            sum /= 5.0
            ranges[i] = sum
        
        ranges = tmp_ranges
    
    def disparity_extender(self, ranges):
        disp_ranges = copy.deepcopy(ranges) # deep copy of ranges
        radius : float32 = vars.CAR_WIDTH / 2

        for i in range(vars.RIGHT, vars.LEFT):
            right_idx : int = i
            left_idx  : int = i + 1

            diff : float32 = ranges[left_idx] - ranges[right_idx]
            theta : float32 = 0.0 # rad

            if abs(diff) > vars.DISP_EPSILON:
                if ranges[left_idx] != 0.0 and diff < 0:  # left side is closer than right side
                    dist : float32 = ranges[left_idx]

                    if abs(radius / dist) < 1:
                        theta = np.arcsin(radius / dist)
                    
                    # disparity extend to the right of corner
                    targ_idx : int = rad_to_idx(idx_to_rad(left_idx) - theta)

                    if targ_idx < vars.RIGHT:
                        targ_idx = vars.RIGHT

                    targ_offs : int = left_idx - targ_idx

                    for j in range(targ_offs):
                        # only extend to given point if current value in disparity range array is farther away
                        if ranges[left_idx] < disp_ranges[right_idx - j]:
                            disp_ranges[right_idx - j] = ranges[left_idx]
                elif ranges[right_idx] != 0.0:         # right side is closer than left side
                    dist : float32 = ranges[right_idx]

                    if abs(radius / dist) < 1:
                        theta = np.arcsin(radius / dist)
                    
                    # disparity extend to the left of corner
                    targ_idx : int = rad_to_idx(idx_to_rad(right_idx) + theta)

                    if targ_idx > vars.LEFT:
                        targ_idx = vars.LEFT

                    targ_offs : int = targ_idx - right_idx

                    for j in range(targ_offs):
                        # only extend to given point if current value in disparity range array is farther away
                        if ranges[right_idx] < disp_ranges[left_idx + j]:
                            disp_ranges[left_idx + j] = ranges[right_idx]

        return disp_ranges

    def closest_idx(self, right_idx, left_idx, ranges):
        min_idx : int = -1
        min_len : float32 = vars.DIST_THRESH

        for i in range(right_idx, left_idx + 1):
            if ranges[i] < min_len and ranges[i] != 0.0:
                min_idx = i
                min_len = ranges[i]

        return min_idx

    def bubble(self, ranges):
        close_idx : int = self.closest_idx(vars.RIGHT, vars.LEFT, ranges)
        close_theta : float32 = idx_to_rad(close_idx)
        close_dist : float32 = ranges[close_idx]

        if close_idx != -1 and close_dist < 1.1:
            theta : float32 = 0.0

            if abs(vars.CAR_WIDTH / (2 * close_dist)) < 1.0:
                theta : float32 = 1.2 * np.arcsin(vars.CAR_WIDTH / (2 * close_dist))

            right_idx : int = rad_to_idx(close_theta - theta)

            if right_idx < vars.RIGHT:
                right_idx = vars.RIGHT

            left_idx  : int = rad_to_idx(close_theta + theta)

            if left_idx > vars.LEFT:
                left_idx = vars.LEFT

            for i in range(right_idx, left_idx + 1):
                ranges[i] = close_dist

    def farthest_idx(self, right_idx, left_idx, ranges):
        right_max : int = right_idx
        left_max  : int = left_idx

        for i in range(right_idx + 1, left_idx + 1):
            if ranges[i] > ranges[right_max]:
                right_max = i
        for i in range(left_idx, right_idx - 1, -1):
            if ranges[i] > ranges[left_max]:
                left_max = i

        return right_max + (left_max - right_max) // 2

    def lidar_callback(self, data):
        """
        Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """

        # deep copy of laser scan message
        new_scan : LaserScan = copy.deepcopy(data)

        # preprocess scan ranges
        self.preprocess_lidar(new_scan.ranges)
        new_scan.ranges = self.disparity_extender(new_scan.ranges)
        self.bubble(new_scan.ranges)
        best_idx : int = self.farthest_idx(vars.RIGHT, vars.LEFT, new_scan.ranges)
        max_idx  : int = self.farthest_idx(vars.RIGHT, vars.LEFT, data.ranges)

        self.laser_pub.publish(new_scan)

        # Determine steering angle from best point, if valid
        angle : float32
        velocity : float32

        if best_idx != -1 and (data.ranges[540] > 2.2 or data.ranges[max_idx] > 3.0):
            angle = idx_to_rad(best_idx)

            if new_scan.ranges[max_idx] > 3.0 and new_scan.ranges[540] > 2.5:
                velocity = 4.0 # m/s
            elif new_scan.ranges[540] > 1.8:
                velocity = 2.0 # m/s
            else:
                velocity = 1.0
        else:
            print("Entering the danger zone... %s" % (np.random.random()))
            best_idx : int = self.farthest_idx(540, 900, new_scan.ranges)
            angle = idx_to_rad(best_idx)

            if data.ranges[vars.LEFT - 210] < 0.8:
                angle = hp.deg_to_rad(-12.5)
            
            velocity = 0.8 # m/s

        # Clamp angle between min and max steering angle
        if angle < hp.deg_to_rad(-20.0):
            angle = hp.deg_to_rad(-20.0)
        elif angle > hp.deg_to_rad(20.0):
            angle = hp.deg_to_rad(20.0)

        #Publish Drive message
        msg = AckermannDriveStamped()
        msg.drive.speed = 1.0
        # msg.drive.speed = velocity
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
