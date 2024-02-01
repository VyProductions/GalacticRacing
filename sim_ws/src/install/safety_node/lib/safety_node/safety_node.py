#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

inf = float("inf")

DST_THRESH : float = 1.00 # minimum meters to obstacle
TTC_THRESH : float = 1.50 # seconds to collision
IDX_STEP : int = 1 # indices between processed scan samples

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
        self.speed = 0.0
        # TODO: create ROS subscribers and publishers.

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 2
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 2
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 2
        )


    def odom_callback(self, odom_msg):
        # TODO: update current speed
        vel : float32 = odom_msg.twist.twist.linear.x
        self.speed = 0.0 if abs(vel) < 0.001 else vel

    def scan_callback(self, scan_msg):
        brake : AckermannDriveStamped() = AckermannDriveStamped()
        ttc : float32 = 0.0
        angle_min : float32 = scan_msg.angle_min
        angle_max : float32 = scan_msg.angle_max
        angle_inc : float32 = scan_msg.angle_increment
        ranges : list = scan_msg.ranges

        for i in range(1080 // IDX_STEP):
            index : int = i * IDX_STEP
            index_rad : float32 = index * angle_inc
            angle_rad : float32 = index_rad + angle_min
            index_deg : float32 = index_rad * (180.0 / np.pi)
            angle_deg : float32 = angle_rad * (180.0 / np.pi)
            scan_dist : float32 = ranges[index]

            if scan_dist < DST_THRESH and self.speed != 0.0:
                print("Breaking -- Minimum distance reached")
                # TODO: publish the command to brake
                self.drive_pub.publish(brake)
            else:
                try:
                    # TODO: compute TTC
                    ttc = inf if self.speed == 0.0 else scan_dist / (self.speed * np.cos(angle_rad))

                    if ttc >= 0.0 and ttc <= 10.0:
                        print("Collision: %.2f m, %.2f sec @ %.2f deg" % (scan_dist, ttc, angle_deg))
                        # TODO: publish the command to brake
                        if ttc <= TTC_THRESH:
                            print("Breaking -- Time to collision threshold crossed")
                            self.drive_pub.publish(brake)
                except Exception as e:
                    print(type(e), e)
            

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()