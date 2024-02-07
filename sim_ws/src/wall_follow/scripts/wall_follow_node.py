#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

float32 = np.float32

# known scan constants
ANGLE_MIN : float32 = -135.0
ANGLE_MAX : float32 =  135.0

# publisher/subscription constants
DRIVE_REFRESH : int = 10  # queries per second (hZ)
SCAN_REFRESH  : int = 10  # queries per second (hZ)

# topic constants
DRIVE_TOPIC : str = "/drive"
SCAN_TOPIC  : str = "/scan"

def clamp(value : float32, min : float32, max : float32) -> float32:
    if (min == max):
        raise ValueError("Clamp: Min and Max in range must be unique.")
    return (value - min) / (max - min)

def deg_to_rad(deg : float32) -> float32:
    return (deg * np.pi) / 180.0

def rad_to_deg(rad : float32) -> float32:
    return (rad * 180.0) / np.pi

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
        self.kp = 14
        self.kd = 0
        self.ki = 0.09

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

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

        lidar_index : int = int(clamp(angle, ANGLE_MIN, ANGLE_MAX) * 1079)
        print(lidar_index)
        return range_data[lidar_index]

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
        left : float32 = self.get_range(range_data, 90.0)
        forward : float32 = self.get_range(range_data, 10.0)

        theta : float32 = np.arctan((forward * np.cos(80.0) - left) / (forward * np.sin(80.0)))
        y : float32 = left * np.cos(theta) - dist
        

        return 0.0

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = 0.0 # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID
        self.get_range(msg.ranges, 0)


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