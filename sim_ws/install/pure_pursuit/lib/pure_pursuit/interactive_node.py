#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import csv
import copy
import atexit
import numpy as np

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

import tf_transformations
from tf_transformations import euler_from_quaternion

server = None
menu_handler = MenuHandler()

root_dir = "/home/vy/GalacticRacing/sim_ws/src"
filename = "levine_blocked"

pathCSV = open(root_dir + "/pure_pursuit/paths/" + filename + ".csv", "w")

def processFeedback(feedback):
    p = feedback.pose.position
    o = feedback.pose.orientation
    print("Marker is now at: (%f, %f, %f) with orientation: (%f, %f, %f, %f)" % (p.x, p.y, p.z, o.x, o.y, o.z, o.w))

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.25
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def normalizeQuaternion( quaternion_msg ):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s

def makeWaypoint(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.pose.position = position
    int_marker.scale = 0.45

    int_marker.name = "Waypoint"

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0

    normalizeQuaternion(control.orientation)

    control.interaction_mode = InteractiveMarkerControl.MENU
    control.name = "Marker Options"
    control.description="Options"
    int_marker.controls.append(copy.deepcopy(control))

    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(copy.deepcopy(control))

    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(control)

    # make a box which also moves in the plane
    control.markers.append( makeBox(int_marker) )
    control.always_visible = True
    int_marker.controls.append(control)

    return int_marker

def saveWaypoint(marker):
    writer = csv.writer(pathCSV, delimiter=',')

    quaternion = np.array([marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w])

    writer.writerow([marker.pose.position.x, marker.pose.position.y, euler_from_quaternion(quaternion)[2]])

class InteractiveMarkerNode(Node):
    def __init__(self):
        super().__init__('interactive_marker_node')

    def placeWaypoint(self, position):
        int_marker = makeWaypoint(position)

        server.insert(int_marker, feedback_callback=processFeedback)

        menu_handler.apply( server, int_marker.name )

        server.applyChanges()

def savePath():
    pathCSV.close()

if __name__ == '__main__':
    atexit.register(savePath)

    rclpy.init()
    print("InteractiveNode Initialized")
    interactive_marker_node = InteractiveMarkerNode()

    server = InteractiveMarkerServer(interactive_marker_node, "i_marker_server")

    menu_handler.insert( "Save Waypoint", callback=saveWaypoint )

    position = Point()
    position.x = 0.0
    position.y = 0.0
    position.z = 0.3

    interactive_marker_node.placeWaypoint(position)

    rclpy.spin(interactive_marker_node)

    interactive_marker_node.destroy_node()
    rclpy.shutdown()