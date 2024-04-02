#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

import atexit
import os.path
from time import gmtime, strftime
from numpy import linalg as LA
import tf_transformations
from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Odometry

root_dir = "/home/vy/GalacticRacing/sim_ws/src"

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger_node')

        # Define subscriber
        self.odom_sub = self.create_subscription(
            Odometry, 'pf/pose/odom', self.save_waypoint, 10
        )

        # Define file system
        self.file = open(strftime(root_dir + '/waypoint_logger/logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')

    def save_waypoint(self, data):
        quaternion = np.array([data.pose.pose.orientation.x, 
                            data.pose.pose.orientation.y, 
                            data.pose.pose.orientation.z, 
                            data.pose.pose.orientation.w])

        euler = euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([data.twist.twist.linear.x, 
                                data.twist.twist.linear.y, 
                                data.twist.twist.linear.z]),2)
        if data.twist.twist.linear.x>0.:
            print(data.twist.twist.linear.x)

        self.file.write('%f, %f, %f, %f, %f, %f, %f, %f\n' % (
            data.pose.pose.position.x, data.pose.pose.position.y,
            data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w,
            euler[2], speed
        ))
    
    def shutdown(self):
        self.file.close()
        print('Goodbye')

def main(args=None):
    rclpy.init(args=args)
    print("Waypoint Logger Initialized")
    waypoint_logger_node = WaypointLogger()

    atexit.register(waypoint_logger_node.shutdown)

    rclpy.spin(waypoint_logger_node)

    waypoint_logger_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
