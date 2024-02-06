#!/usr/bin/env python3

# nodes
import rclpy
from rclpy.node import Node

# mathematics
import numpy as np

# ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# publisher/subscription constants
DRIVE_REFRESH : int = 10  # queries per second (hZ)
SCAN_REFRESH  : int = 10  # queries per second (hZ)
ODOM_REFRESH  : int = 10  # queries per second (hZ)

# topic constants
DRIVE_TOPIC : str = "/drive"
ODOM_TOPIC  : str = "/ego_racecar/odom"
SCAN_TOPIC  : str = "/scan"

# odometry constants
SPEED_EPSILON : float = 0.001  # smallest meaningful speed difference [+/- 1 millimeter/sec]

# TTC constants
BRAKE : AckermannDriveStamped() = AckermannDriveStamped()  # published message to apply brakes
DST_THRESH : float = 1.00  # minimum meters to obstacle
TTC_THRESH : float = 1.50  # seconds to collision
INDEX_STEP : int   = 1     # indices between processed scan samples

# math constants
PI  = np.pi         # pi
INF = float("inf")  # infinity

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        # vehicle properties
        self.speed = 0.0
        self.brake_i = 0
        self.braking_coeffs : list = [0.0, 0.33, 0.5, 1.0]

        # topic publishers
        self.drive_pub = self.create_publisher(  # drive topic publisher
            AckermannDriveStamped, DRIVE_TOPIC, DRIVE_REFRESH
        )

        # topic subscribers
        self.odom_sub = self.create_subscription(  # odometry topic subscription
            Odometry, ODOM_TOPIC, self.odom_callback, ODOM_REFRESH
        )

        self.scan_sub = self.create_subscription(  # laser scan topic subscription
            LaserScan, SCAN_TOPIC, self.scan_callback, SCAN_REFRESH
        )


    def odom_callback(self, odom_msg):
        # get current speed
        vel : float32 = odom_msg.twist.twist.linear.x

        # set speed, or zero out if below minimum meaningful speed
        self.speed = 0.0 if abs(vel) < SPEED_EPSILON else vel

    def scan_callback(self, scan_msg):
        # scan fields
        ranges : list = scan_msg.ranges                 # list of 1080 scan samples
        range_min : float32 = scan_msg.range_min        # minimum scannable range
        range_max : float32 = scan_msg.range_max        # maximum scannable range
        angle_min : float32 = scan_msg.angle_min        # minimum scannable angle
        angle_inc : float32 = scan_msg.angle_increment  # radians between scan sample 'k' and 'k+1'

        # TTC values
        ttc : float32 = 0.0  # time (seconds) until collision with an obstacle

        for i in range(1080 // INDEX_STEP):
            index : int = i * INDEX_STEP                         # sample indice in scan ranges array
            scan_dist : float32 = ranges[index]                  # distance acquired at given index
            angle_rad : float32 = index * angle_inc + angle_min  # computed theta angle relative to forward (locally x-oriented) direction

            # verify that scanned distance is a valid sample
            if scan_dist > range_min and scan_dist < range_max:
                try:
                    # compute time to collision
                    ttc = INF if self.speed == 0.0 else scan_dist / (self.speed * np.cos(angle_rad))

                    # filter impending collisions for printing
                    # if ttc >= TTC_THRESH and ttc <= 5.0:
                    #     print("Collision: %.2f m, %.2f sec @ %.2f deg" % (scan_dist, ttc, angle_rad * (180.0 / PI)))

                    # publish the command to brake if moving towards obstacle and going to collide in under 'TTC_THRESH' seconds
                    if ttc >= 0.0 and ttc <= TTC_THRESH:
                        self.drive_pub.publish(BRAKE)
                except Exception as e:
                    print(type(e), e)
            

def main(args=None):
    rclpy.init(args=args)
    print("Safety Node Initialized")
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()