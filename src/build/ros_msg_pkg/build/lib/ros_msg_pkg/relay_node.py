#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class RelayNode(Node):
    def __init__(self):
        super().__init__("relay_node")
        self.speed = 0.0
        self.steering_angle = 0.0
        #Create /drive subscriber
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, 'drive', self.drive_callback, 10
        )

        #Create /relay_drive publisher
        self.relay_drive_pub= self.create_publisher(
            AckermannDriveStamped, 'relay_drive', 10
        )
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def drive_callback(self,drive_msg):
        self.speed = 3 * drive_msg.drive.speed
        self.steering_angle = 3 * drive_msg.drive.steering_angle

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.relay_drive_pub.publish(msg)
        self.get_logger().info('Publishing to /relay_drive[%s}]: speed=%s , steering_angle=%s'%(self.i,msg.drive.speed,msg.drive.
        steering_angle))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
