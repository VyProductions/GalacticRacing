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
        gap_scan_topic = '/gap_scan'

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

        self.laser_gap_pub = self.create_publisher(
            LaserScan, gap_scan_topic, vars.SCAN_REFRESH
        )

        self.velocity : float32 = 0.0
        self.steering_angle : float32 = 0.0
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        for i in range(vars.RANGE_SIZE):
            if ranges[i] > vars.DIST_THRESH:
                ranges[i] = vars.DIST_THRESH
            elif ranges[i] == float("nan"):
                ranges[i] = 0.0
            # force occlusion for gaps that are quite small
            elif i > vars.SAMPLE_RADIUS and i < vars.RANGE_SIZE - vars.SAMPLE_RADIUS and \
                ranges[i] > ranges[i - vars.SAMPLE_RADIUS] + vars.DISP_EPSILON and \
                ranges[i] > ranges[i + vars.SAMPLE_RADIUS] + vars.DISP_EPSILON:
                ranges[i] = ranges[i - vars.SAMPLE_RADIUS]

        # perform a kerneling operation to average out each point over the kernel
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

        for i in range(vars.RIGHT + 0, vars.LEFT - 0):
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
                elif ranges[right_idx] != 0.0:  # right side is closer than left side
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

        disp_list : List[int] = []

        for i in range(vars.RIGHT, vars.LEFT + 1):
            right_idx : int = i
            left_idx  : int = i + 1
            diff : float32 = disp_ranges[left_idx] - disp_ranges[right_idx]

            if abs(diff) > vars.DISP_EPSILON:
                disp_list.append(i)


        return disp_ranges, disp_list

    def closest_idx(self, right_idx, left_idx, ranges):
        min_idx : int = -1
        min_dist :float32 = float("inf")

        for i in range(right_idx, left_idx + 1):
            dist : float32 = ranges[i]
            if dist < min_dist and dist != 0.0:
                min_idx = i
                min_dist = dist

        return min_idx

    def bubble(self, ranges):
        close_idx1 : int = self.closest_idx(135, 540, ranges)
        close_idx2 : int = self.closest_idx(540, 944, ranges)
        close_theta1 : float32 = idx_to_rad(close_idx1)
        close_theta2 : float32 = idx_to_rad(close_idx2)
        close_dist1 : float32 = ranges[close_idx1]
        close_dist2 : float32 = ranges[close_idx2]
        radius : float32 = vars.CAR_WIDTH / 2

        if close_idx1 != -1 and close_dist1 < 3.0:
            theta : float32 = 0.0

            if abs(vars.CAR_WIDTH / (2 * close_dist1)) < 1.0:
                theta : float32 = vars.BUBBLE_MULT * np.arcsin(radius / close_dist1)

            right_idx : int = rad_to_idx(close_theta1 - theta)

            if right_idx < vars.RIGHT:
                right_idx = vars.RIGHT

            left_idx  : int = rad_to_idx(close_theta1 + theta)

            if left_idx > vars.LEFT:
                left_idx = vars.LEFT

            for i in range(right_idx, left_idx + 1):
                ranges[i] = close_dist1
        
        if close_idx2 != -1 and close_dist2 < 3.0:
            theta : float32 = 0.0

            if abs(vars.CAR_WIDTH / (2 * close_dist2)) < 1.0:
                theta : float32 = vars.BUBBLE_MULT * np.arcsin(radius / close_dist2)

            right_idx : int = rad_to_idx(close_theta2 - theta)

            if right_idx < vars.RIGHT:
                right_idx = vars.RIGHT

            left_idx  : int = rad_to_idx(close_theta2 + theta)

            if left_idx > vars.LEFT:
                left_idx = vars.LEFT

            for i in range(right_idx, left_idx + 1):
                ranges[i] = close_dist2

    def farthest_idx(self, right_idx, left_idx, ranges):
        right_max : int = right_idx

        for i in range(right_idx + 1, left_idx + 1):
            if ranges[i] > ranges[right_max]:
                right_max = i

        left_max  : int = right_max

        for i in range(right_max + 1, left_idx + 1):
            if abs(ranges[left_max] - ranges[right_max]) < 0.1:
                left_max = i
            else:
                break
        
        prev_right_max : int = right_max
        tmp : int = right_max

        for i in range(right_max - 1, right_idx - 1, -1):
            if abs(ranges[tmp] - ranges[right_max]) < 0.1:
                tmp = i
            else:
                break
        
        right_max = tmp

        if left_max - right_max > 10:
            return right_max + (left_max - right_max) // 2
        else:
            return prev_right_max

    def lidar_callback(self, data):
        """
        Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """

        # deep copy of laser scan message
        new_scan : LaserScan = copy.deepcopy(data)
        gap_scan : LaserScan = copy.deepcopy(data)

        # preprocess scan ranges
        self.preprocess_lidar(new_scan.ranges)
        new_scan.ranges, disp_list = self.disparity_extender(new_scan.ranges)
        self.bubble(new_scan.ranges)

        close_idx : int = self.closest_idx(510, 570, data.ranges)

        self.laser_pub.publish(new_scan)

        best_idx : int = self.farthest_idx(vars.RIGHT, vars.LEFT, new_scan.ranges)

        # Set velocity based on forward room
        if len(disp_list) == 0:
            peek_left_idx = self.farthest_idx(vars.LEFT, vars.PEEK_LEFT, data.ranges)
            peek_right_idx = self.farthest_idx(vars.PEEK_RIGHT, vars.RIGHT, data.ranges)
            l_dist = data.ranges[peek_left_idx]
            r_dist = data.ranges[peek_right_idx]

            best_idx = peek_left_idx if l_dist > r_dist else peek_right_idx

            self.velocity = 0.5
        elif data.ranges[480] < 0.8 or data.ranges[600] < 0.8:
            self.velocity = 0.8
        elif data.ranges[best_idx] > 4.5:
            self.velocity = 3.5
        elif data.ranges[best_idx] > 3.5:
            self.velocity = 2.5
        elif data.ranges[best_idx] > 2.5:
            self.velocity = 1.5
        elif data.ranges[best_idx] > 1.5:
            self.velocity = 1.0
        elif data.ranges[best_idx] > 0.8:
            self.velocity = 0.8
        elif data.ranges[best_idx] > 0.3:
            self.velocity = 0.5
        else:
            self.velocity = 0.0

        self.steering_angle = idx_to_rad(best_idx)

        if (
            self.steering_angle < 0.0 and (
                np.sin(hp.deg_to_rad(55.0)) * new_scan.ranges[40] < vars.CAR_WIDTH/2 + 0.035 or
                new_scan.ranges[180] < vars.CAR_WIDTH/2 + 0.04 or
                np.sin(np.pi/4) * new_scan.ranges[360] < vars.CAR_WIDTH/2 - 0.05
            )
        ):
            self.steering_angle = hp.deg_to_rad(1.5)
            self.velocity = 0.5
        elif (
            self.steering_angle > 0.0 and (
                np.sin(hp.deg_to_rad(55.0)) * new_scan.ranges[1039] < vars.CAR_WIDTH/2 + 0.041 or
                new_scan.ranges[899] < vars.CAR_WIDTH/2 + 0.041 or
                np.sin(np.pi/4) * new_scan.ranges[720] < vars.CAR_WIDTH/2 - 0.05
            )
        ):
            self.steering_angle = hp.deg_to_rad(-1.75)
            self.velocity = 0.5

        # Clamp angle between min and max steering angle
        if self.steering_angle < hp.deg_to_rad(-20.0):
            self.steering_angle = hp.deg_to_rad(-20.0)
        elif self.steering_angle > hp.deg_to_rad(20.0):
            self.steering_angle = hp.deg_to_rad(20.0)

        #Publish Drive message
        msg = AckermannDriveStamped()
        # msg.drive.speed = 0.5
        msg.drive.speed = self.velocity
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
