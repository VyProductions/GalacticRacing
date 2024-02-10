#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# import time # time.process_time() for dt computations

float32 = np.float32

def deg_to_rad(deg : float32) -> float32:
    return (deg * np.pi) / 180.0

def rad_to_deg(rad : float32) -> float32:
    return (rad * 180.0) / np.pi

# publisher/subscription constants
DRIVE_REFRESH : int = 10  # queries per second (hZ)
SCAN_REFRESH  : int = 10  # queries per second (hZ)

# topic constants
DRIVE_TOPIC : str = "/drive"
SCAN_TOPIC  : str = "/scan"

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')
        
        # publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, DRIVE_TOPIC, DRIVE_REFRESH
        )

        # subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, SCAN_TOPIC, self.scan_callback, SCAN_REFRESH
        )

        # set PID gains
        self.kp = 1.0
        # self.ki = 0.0
        # self.kd = 0.0

        # set velocity values
        self.max_speed = 4.0
        self.turn_speed = 1.25

        # scan values
        self.MIN_RANGE = 0.0
        self.MAX_RANGE = 30.0

        # error computation values
        self.left_rads = deg_to_rad(90)
        self.forwardleft_rads = deg_to_rad(34.5)
        self.forwardright_rads = -self.forwardleft_rads
        self.frontleft_rads = deg_to_rad(5)
        self.frontright_rads = -self.frontleft_rads
        self.projected = 1.3
        self.distance = 0.9

        # store history
        # self.integral = 0.0
        # self.prev_error = 0.0
        # self.error = 0.0
        # self.last_time = time.process_time()

        # store current details
        self.speed = 0.0
        self.steering_angle = 0.0
        # self.dt = 0.0

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        lidar_index : int = int(np.round((angle + deg_to_rad(135.0)) * (1080/deg_to_rad(270.0))))
        range = range_data[lidar_index]
        return range if range > self.MIN_RANGE and range < self.MAX_RANGE else float("inf")

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        # angular distance between perpendicular left and main forward left sample
        diff_rad : float32 = self.left_rads - self.forwardleft_rads

        # distances along select rays to obstacles
        left : float32 = self.get_range(range_data, self.left_rads)
        forwardL : float32 = self.get_range(range_data, self.forwardleft_rads)
        forwardR : float32 = self.get_range(range_data, self.forwardright_rads)
        frontL : float32 = self.get_range(range_data, self.frontleft_rads)
        frontR : float32 = self.get_range(range_data, self.frontright_rads)

        # estimate distance to the left wall
        theta : float = -np.arctan((forwardL * np.cos(diff_rad) - left) / (forwardL * np.sin(diff_rad)))
        act_dist : float = left * np.cos(theta)
        proj_dist : float = act_dist - self.projected * np.sin(theta)

        # self.dt = time.process_time() - self.last_time
        # self.last_time = time.process_time()

        # compute error to turn from
        error = proj_dist - dist

        # if going to crash
        if frontL < 2.0 and forwardL < 2.2 and forwardL < forwardR:
            # dont
            error -= 0.2 # forcefully turn right
        elif frontR < 2.0 and forwardR < 2.2 and forwardL > forwardR:
            # also dont
            error += 0.2 # forcefully turn left

        return error

    def pid_control(self, error, velocity):
        print(f"Error: {error}")
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # TODO: Use kp, ki & kd to implement a PID controller
        U : float = (
            self.kp * error # +
            # self.ki * self.integral +
            # self.kd * (self.error - self.prev_error)
        )
        # self.prev_error = error
        
        if U < deg_to_rad(-20.0):
            self.steering_angle = deg_to_rad(-20.0)
        elif U > deg_to_rad(20.0):
            self.steering_angle = deg_to_rad(20.0)
        else:
            self.steering_angle = U

        # fill in drive message and publish
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = self.steering_angle
        drive_msg.drive.speed = velocity
        self.speed = 0.0
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # get max/min range of laser scan for distance verification
        self.MIN_RANGE = msg.range_min
        self.MAX_RANGE = msg.range_max

        # get error for steering
        error = self.get_error(msg.ranges, self.distance)

        # adjust speed according to steering angle
        velocity = self.max_speed if abs(self.steering_angle) >= 0.0 and \
            abs(self.steering_angle) < deg_to_rad(10.0) else self.turn_speed
        
        # actuate the car with PID
        self.pid_control(error, velocity)


def main(args=None):
    rclpy.init(args=args)
    print("Wall Follow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()