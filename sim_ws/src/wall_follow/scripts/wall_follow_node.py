#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import time

float32 = np.float32

def deg_to_rad(deg : float32) -> float32:
    return (deg * np.pi) / 180.0

def rad_to_deg(rad : float32) -> float32:
    return (rad * 180.0) / np.pi

# known scan constants
ANGLE_MIN : float32 = deg_to_rad(-135.0)
ANGLE_MAX : float32 = deg_to_rad(135.0)

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
        # TODO: create subscribers and publishers
        
        # publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, DRIVE_TOPIC, DRIVE_REFRESH
        )

        # subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, SCAN_TOPIC, self.scan_callback, SCAN_REFRESH
        )

        # TODO: set PID gains
        self.kp = 2.0
        self.ki = 0.0
        self.kd = 0.0

        # TODO: set velocity values
        self.max_speed = 2.0
        self.turn_speed = 0.5

        # TODO: error computation values
        self.left_rads = deg_to_rad(90)
        self.forward_rads = deg_to_rad(5)
        self.projected = self.max_speed / DRIVE_REFRESH
        self.distance = 0.5

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.last_time = time.process_time()

        # TODO: store current stats
        self.speed = 0.0
        self.dt = 0.0
        self.steering_angle = 0.0

        # TODO: store any necessary values you think you'll need

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
        return range if range >= 0.0 and range <= 30.0 else 30.0

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        diff_rad : float32 = self.left_rads - self.forward_rads

        left : float32 = self.get_range(range_data, self.left_rads)
        forward : float32 = self.get_range(range_data, self.forward_rads)

        theta : float = -np.arctan((forward * np.cos(diff_rad) - left) / (forward * np.sin(diff_rad)))
        act_dist : float = left * np.cos(theta)
        proj_dist : float = act_dist - self.projected * np.sin(theta)

        self.dt = time.process_time() - self.last_time
        self.last_time = time.process_time()

        return proj_dist - dist

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
            self.kp * error +
            self.ki * self.integral +
            self.kd * (self.prev_error - self.error) / self.dt
        )
        self.integral += error
        self.prev_error = error

        print(f"U (rad): {np.round(U, 2)}, min (rad): {np.round(deg_to_rad(-20.0), 2)}, max (rad): {np.round(deg_to_rad(20.0), 2)}")
        
        if U < deg_to_rad(-20.0):
            self.steering_angle = deg_to_rad(-20.0)
        elif U > deg_to_rad(20.0):
            self.steering_angle = deg_to_rad(20.0)
        else:
            self.steering_angle = U

        # TODO: fill in drive message and publish
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
        error = self.get_error(msg.ranges, self.distance) # TODO: replace with error calculated by get_error()
        velocity = self.max_speed if abs(self.steering_angle) >= 0.0 and abs(self.steering_angle) < deg_to_rad(10.0) else self.turn_speed
        self.pid_control(error, velocity) # TODO: actuate the car with PID


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