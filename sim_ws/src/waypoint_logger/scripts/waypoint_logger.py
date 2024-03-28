#!/usr/bin/env python
import rclpy
from rclpy.node import Node

import numpy as np
import atexit
import tf2
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

home = expanduser('~')
file = open(strftime(home+'/rcws/logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')

def save_waypoint(data):
    quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])

    euler = tf.transformations.euler_from_quaternion(quaternion)
    speed = LA.norm(np.array([data.twist.twist.linear.x, 
                              data.twist.twist.linear.y, 
                              data.twist.twist.linear.z]),2)
    if data.twist.twist.linear.x>0.:
        print(data.twist.twist.linear.x)

    file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                     data.pose.pose.position.y,
                                     euler[2],
                                     speed))

def shutdown():
    file.close()
    print('Goodbye')
 
def listener():
    rclpy.init_node('waypoints_logger', anonymous=True)
    rclpy.Subscriber('pf/pose/odom', Odometry, save_waypoint)
    rclpy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving waypoints...')
    try:
        listener()
    except rclpy.ROSInterruptException:
        pass

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger_node')

def main(args=None):
    rclpy.init(args=args)
    print("Gap Follow Initialized")
    reactive_node = WaypointLogger()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()