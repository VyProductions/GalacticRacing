#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import copy
from interactive_markers import InteractiveMarkerServer
from geometry_msgs.msg import Point
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

class InteractiveMarkerNode(Node):
    def __init__(self):
        super().__init__('interactive_marker_node')

        self.server = InteractiveMarkerServer(self, "i_marker_server")

    def placeNode(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "moving_frame"
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 1.0
        int_marker.scale = 1.0

        int_marker.name = "moving"
        int_marker.description = "Marker Attached to a\nMoving Frame"

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.always_visible = True
        control.markers.append( makeBox(int_marker) )
        int_marker.controls.append(control)

        self.server.insert(int_marker, feedback_callback=self.processFeedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()
    
    def processFeedback(self, feedback):
        p = feedback.pose.position
        print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))

def main(args=None):
    rclpy.init(args=args)
    print("InteractiveNode Initialized")
    interactive_marker_node = InteractiveMarkerNode()

    interactive_marker_node.placeNode()

    rclpy.spin(interactive_marker_node)

    interactive_marker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()