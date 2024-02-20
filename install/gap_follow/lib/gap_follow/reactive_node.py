#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

float32 = np.float32

def deg_to_rad(deg : float32) -> float32:
    return (deg * np.pi) / 180.0

def rad_to_deg(rad : float32) -> float32:
    return (rad * 180.0) / np.pi

# car dimensions
CAR_WIDTH  : float32 = 0.30 # meters
CAR_LENGTH : float32 = 0.45 # meters

# refresh frequencies
SCAN_REFRESH = 10
DRIVE_REFRESH = 10

# scan preprocessing constants
RANGE_SIZE : int = 1080
WIND_SIZE  : int = 5
FOV : float = deg_to_rad(270.0)

# distance processing
DIST_THRESH  : float32 = 3.00 # meters
DISP_EPSILON : float32 = 0.40 # meters

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

        # subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, lidarscan_topic, self.lidar_callback, SCAN_REFRESH
        )

        # publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, DRIVE_REFRESH
        )

        self.max_speed  = 1.0 # m/s
        self.turn_speed = 0.5 # m/s
        self.steering_angle = 0.0 # rads relative to forward (left: > 0.0, right: < 0.0)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = [None] * RANGE_SIZE

        wind_rad  : int = WIND_SIZE // 2
        proc_size : int = RANGE_SIZE - 2 * wind_rad

        for i in range(proc_size):
            sum : float32 = 0.0

            for j in range(WIND_SIZE):
                if ranges[i + j] < 0.0:
                    ranges[i + j] = 0.0

                sum += ranges[i + j]

            sum = np.round(sum / float(WIND_SIZE), 2)

            if sum > DIST_THRESH:
                sum = float("inf")  # too far away; make it infinity
            
            proc_ranges[i + wind_rad] = sum

        # duplicate average values into edge of ranges array
        for i in range(wind_rad):
            proc_ranges[i] = proc_ranges[wind_rad]
            proc_ranges[RANGE_SIZE - i - 1] = proc_ranges[RANGE_SIZE - wind_rad - 1]

        return proc_ranges

    def disparity_extender(self, proc_ranges):
        # step 2: find all disparities
        for i in range(RIGHT, LEFT):
            diff :float32 = proc_ranges[i] - proc_ranges[i + 1]
            # step 3: extend disparities
            if abs(diff) > DISP_EPSILON:
                if diff < 0: # left point is farther away
                    theta : float32 = np.arcsin(CAR_WIDTH / (2 * proc_ranges[i]))
                    targ_idx : int = rad_to_idx(idx_to_rad(i) + theta)

                    # extend disparities left
                    for j in range(i + 1, targ_idx + 1):
                        if proc_ranges[j] > proc_ranges[i]:
                            proc_ranges[j] = proc_ranges[i]
                else: # right point is farther away
                    theta : float32 = np.arcsin(CAR_WIDTH / (2 * proc_ranges[i + 1]))
                    targ_idx : int = rad_to_idx(idx_to_rad(i + 1) - theta)

                    # extend disparities right
                    for j in range(i, targ_idx - 1, -1):
                        if proc_ranges[j] > proc_ranges[i + 1]:
                            proc_ranges[j] = proc_ranges[i + 1]
        
        # step 4: find largest distance in ranges after extending disparities
        largest_idx : int = RIGHT

        for i in range(RIGHT + 1, LEFT + 1):
            if proc_ranges[i] > proc_ranges[largest_idx]:
                largest_idx = i

        return largest_idx

    def lidar_callback(self, data):
        """
        Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges

        # preprocess range data
        proc_ranges = self.preprocess_lidar(ranges)

        # Get next index using disparity extender algorithm
        best_idx = self.disparity_extender(proc_ranges)

        # Determine steering angle from best point
        angle = idx_to_rad(best_idx)

        # Clamp angle between min and max steering angle
        if angle < deg_to_rad(-20.0):
            angle = deg_to_rad(-20.0)
        elif angle > deg_to_rad(20.0):
            angle = deg_to_rad(20.0)

        print(f"Steering to angle: {rad_to_deg(angle)}")
        self.steering_angle = angle

        # Limit velocity for turning
        velocity = self.max_speed if abs(self.steering_angle) >= 0.0 and \
            abs(self.steering_angle) < deg_to_rad(10.0) else self.turn_speed
        # velocity = 0.0

        #Publish Drive message
        msg = AckermannDriveStamped()
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
