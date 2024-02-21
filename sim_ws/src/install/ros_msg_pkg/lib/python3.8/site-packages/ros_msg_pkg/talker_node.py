#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class TalkerNode(Node):
    def __init__(self):
        super().__init__("talker_node")
        self.declare_parameter('v', 0.2)
        self.declare_parameter('d', 0.1)
        self.speed = self.get_parameter('v').value
        self.steering_angle = self.get_parameter('d').value

        #Create /drive publisher
        self.drive_pub= self.create_publisher(
            AckermannDriveStamped, 'drive', 10
        )
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info("initial speed=%s , initial steering_angle=%s"%(self.speed,self.steering_angle))

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.drive_pub.publish(msg)
        self.get_logger().info('Publishing to /drive[%s}]: speed=%s , steering_angle=%s'%(self.i,msg.drive.speed,msg.drive.steering_angle))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
