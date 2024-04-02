#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import csv
import copy
import math
import time
import numpy as np

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion, Pose
from scipy.interpolate import interp1d, CubicSpline

from tf_transformations import *

server = None
menu_handler = MenuHandler()

root_dir = "/home/vy/GalacticRacing/sim_ws/src"
filename = "levine_blocked"

def makeBox():
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.r = 0.0
    marker.color.g = 0.4
    marker.color.b = 0.8
    marker.color.a = 0.2

    return marker

def createMarker(position, orientation, id):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.pose.position = position
    int_marker.pose.orientation = orientation
    int_marker.scale = 0.45

    int_marker.name = "Waypoint_" + str(id)

    control = InteractiveMarkerControl()
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0
    control.orientation.w = 1.0

    control.interaction_mode = InteractiveMarkerControl.MENU
    control.name = "Marker Options"
    control.description = int_marker.name
    int_marker.controls.append(copy.deepcopy(control))

    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(control)

    # make a box which also moves in the plane
    control.markers.append(makeBox())
    control.always_visible = True
    int_marker.controls.append(control)

    return int_marker

class InteractiveMarkerNode(Node):
    def __init__(self):
        super().__init__('interactive_marker_node')

        # Define publishers
        self.markers_pub = self.create_publisher(
            MarkerArray, "/visualization_marker_array", 10
        )

        self.path = []
        self.markers = {}
        self.position = Point(x=0.0, y=0.0, z=0.3)
        self.orientation = self.getQuart(0.0)
        self.marker_count = 0

    def getQuart(self, angle):
        quaternion = [0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2)]

        rotated_quaternion = Quaternion()
        rotated_quaternion.x = quaternion[0]
        rotated_quaternion.y = quaternion[1]
        rotated_quaternion.z = quaternion[2]
        rotated_quaternion.w = quaternion[3]

        return rotated_quaternion

    def processFeedback(self, feedback):
        p = feedback.pose.position
        o = feedback.pose.orientation

        self.markers[feedback.marker_name]["position"] = p
        self.markers[feedback.marker_name]["orientation"] = o
        self.markers[feedback.marker_name]["rot"] = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        
        # print(f"{feedback.marker_name} is now at: ({p.x}, {p.y}, {p.z}) with orientation: ({o.x}, {o.y}, {o.z}, {o.w})")

        if feedback.control_name == "Marker Options_u2":
            self.renderPath()

    # marker options

    def newMarker(self, marker=None):
        rot = 0.0 if marker==None else self.markers[marker.marker_name]["rot"]

        int_marker = createMarker(
            self.position if marker == None else self.markers[marker.marker_name]["position"],
            self.orientation if marker == None else self.markers[marker.marker_name]["orientation"],
            self.marker_count
        )

        self.markers[int_marker.name] = {
            "position": int_marker.pose.position,
            "orientation": int_marker.pose.orientation,
            "rot": rot,
            "speed": 1.0 if marker == None else self.markers[marker.marker_name]["speed"],
            "lookahead": 1.0 if marker == None else self.markers[marker.marker_name]["lookahead"]
        }

        self.marker_count += 1

        server.insert(int_marker, feedback_callback=self.processFeedback)

        menu_handler.apply(server, int_marker.name)

        server.applyChanges()

        self.renderPath()

    def details(self, marker):
        pos = self.markers[marker.marker_name]["position"]

        print(f"Details: {marker.marker_name}")
        print("  Position:  (%.2f, %.2f)" % (pos.x, pos.y))
        print(f"  Rotation:  {math.degrees(self.markers[marker.marker_name]['rot'])}")
        print(f"  Speed:     {self.markers[marker.marker_name]['speed']} m/s")
        print(f"  Lookahead: {self.markers[marker.marker_name]['lookahead']} m")

    def deleteMarker(self, marker):
        if len(self.markers) > 1:
            del self.markers[marker.marker_name]
            server.erase(marker.marker_name)
            server.applyChanges()

            self.renderPath()
    
    def loadMarkers(self, marker):
        filename = input("Markers input filename? ")

        markerCSV = open(root_dir + "/pure_pursuit/markers/" + filename, "r")

        reader = csv.reader(markerCSV, delimiter=',')

        for name in self.markers:
            server.erase(name)

        server.applyChanges()

        self.markers = {}
        self.marker_count = 0

        for row in reader:
            self.markers["Waypoint_" + str(self.marker_count)] = {
                "position": Point(x=float(row[0]), y=float(row[1]), z=0.3),
                "orientation": Quaternion(x=float(row[2]), y=float(row[3]), z=float(row[4]), w=float(row[5])),
                "rot": float(row[6]),
                "speed": float(row[7]),
                "lookahead": float(row[8])
            }

            int_marker = createMarker(
                self.markers["Waypoint_" + str(self.marker_count)]["position"],
                self.markers["Waypoint_" + str(self.marker_count)]["orientation"],
                self.marker_count
            )

            self.marker_count += 1

            server.insert(int_marker, feedback_callback=self.processFeedback)

            menu_handler.apply(server, int_marker.name)

        server.applyChanges()

        markerCSV.close()

        self.renderPath()
    
    def saveMarkers(self, marker):
        filename = input("Markers output filename? ")

        markerCSV = open(root_dir + "/pure_pursuit/markers/" + filename, "w")

        writer = csv.writer(markerCSV, delimiter=',')

        for name, marker in self.markers.items():
            pos = marker["position"]
            orient = marker["orientation"]

            writer.writerow([
                pos.x, pos.y,
                orient.x, orient.y, orient.z, orient.w,
                marker["rot"], marker["speed"], marker["lookahead"]
            ])

        markerCSV.close()

    # alignment options
            
    def alignX(self, marker):
        marker_name = input("Other marker's name: ")

        if marker_name != marker.marker_name and marker_name in self.markers:
            self.markers[marker.marker_name]["position"].x = self.markers[marker_name]["position"].x

            pose = Pose()
            pose.position.x = self.markers[marker.marker_name]["position"].x
            pose.position.y = self.markers[marker.marker_name]["position"].y
            pose.position.z = self.markers[marker.marker_name]["position"].z
            pose.orientation = self.markers[marker.marker_name]["orientation"]

            server.setPose(marker.marker_name, pose=pose)
            server.applyChanges()

            print(f"Aligned {marker.marker_name} to the x-position of {marker_name}.")

            self.renderPath()
            
    def alignY(self, marker):
        marker_name = input("Other marker's name: ")

        if marker_name != marker.marker_name and marker_name in self.markers:
            self.markers[marker.marker_name]["position"].y = self.markers[marker_name]["position"].y

            pose = Pose()
            pose.position.x = self.markers[marker.marker_name]["position"].x
            pose.position.y = self.markers[marker.marker_name]["position"].y
            pose.position.z = self.markers[marker.marker_name]["position"].z
            pose.orientation = self.markers[marker.marker_name]["orientation"]

            server.setPose(marker.marker_name, pose=pose)
            server.applyChanges()

            print(f"Aligned {marker.marker_name} to the y-position of {marker_name}.")

            self.renderPath()

    def rotationAlignment(self, marker):
        rot = self.markers[marker.marker_name]["rot"]
        nearest_div_15 = round((rot * 12) / np.pi)
        nearest_15 = (nearest_div_15 * np.pi) / 12
        self.markers[marker.marker_name]["rot"] = nearest_15

        quaternion = self.getQuart(nearest_15)

        pose = Pose()
        pose.position.x = marker.pose.position.x
        pose.position.y = marker.pose.position.y
        pose.position.z = marker.pose.position.z

        pose.orientation.x = quaternion.x
        pose.orientation.y = quaternion.y
        pose.orientation.z = quaternion.z
        pose.orientation.w = quaternion.w

        server.setPose(marker.marker_name, pose=pose)
        server.applyChanges()

        print(f"Aligned {marker.marker_name} to the nearest 15 degree rotation.")
    
    # waypoint options

    def setSpeed(self, marker):
        new_speed = input("New Speed [0.5m/s - 20.0m/s]: ")

        try:
            f_speed = float(new_speed)

            if f_speed >= 0.5 and f_speed <= 20.0:
                self.markers[marker.marker_name]["speed"] = f_speed
                print(f"  Speed set to {self.markers[marker.marker_name]['speed']} m/s.")

                self.renderPath()
        except ValueError:
            print("Invalid input. Try again.")
    
    def setLookahead(self, marker):
        new_lookahead = input("New Lookahead [0.2m - 10.0m]: ")

        try:
            f_lookahead = float(new_lookahead)

            if f_lookahead >= 0.2 and f_lookahead <= 10.0:
                self.markers[marker.marker_name]["lookahead"] = f_lookahead
                print(f"  Lookahead set to {self.markers[marker.marker_name]['lookahead']} m.")

                self.renderPath()
        except ValueError:
            print("Invalid input. Try again.")

    # path options
        
    def renderPath(self, marker=None):
        self.deletePath(marker)

        x = np.array([marker["position"].x for name, marker in self.markers.items()])
        y = np.array([marker["position"].y for name, marker in self.markers.items()])
        speed = np.array([marker['speed'] for name, marker in self.markers.items()])
        lookahead = np.array([marker['lookahead'] for name, marker in self.markers.items()])

        x = np.concatenate(([x[-1]], x, [x[0]]))
        y = np.concatenate(([y[-1]], y, [y[0]]))
        speed = np.concatenate(([speed[-1]], speed, [speed[0]]))
        lookahead = np.concatenate(([lookahead[-1]], lookahead, [lookahead[0]]))

        # find total linear arc length along path at each point
        # distance at current point is distance of previous point plus euclidean distance between previous point and current point

        num_pts = len(x)

        cumulative_dists = [0.0] * num_pts

        for i in range(1, num_pts):
            cumulative_dists[i] = cumulative_dists[i-1] + ((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)**0.5
        
        if cumulative_dists[-1] > 0.0:
            norm_dists = [dist / cumulative_dists[-1] for dist in cumulative_dists]

            x_interp = CubicSpline(norm_dists, x, bc_type="natural")
            y_interp = CubicSpline(norm_dists, y, bc_type="natural")
            speed_interp = interp1d(norm_dists, speed)
            lookahead_interp = interp1d(norm_dists, lookahead)

            t = np.linspace(cumulative_dists[1] / cumulative_dists[-1], cumulative_dists[-1] / cumulative_dists[-1], int(cumulative_dists[-1] * 10))

            self.path = []
            markers = MarkerArray()

            j = 0

            for _, i in enumerate(t):
                marker = Marker()

                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "ns_interpmarkers"

                marker.id = j
                marker.type = 1
                marker.action = 0

                marker.pose.position.x = float(x_interp(i))
                marker.pose.position.y = float(y_interp(i))
                marker.pose.position.z = 0.2

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                self.path.append({
                    'x': float(x_interp(i)),
                    'y': float(y_interp(i)),
                    'rot': 0.0,
                    'speed': float(speed_interp(i)),
                    'lookahead': float(lookahead_interp(i))
                })

                markers.markers.append(marker)

                j += 1

            self.markers_pub.publish(markers)

    def deletePath(self, marker):
        self.path = []
        markers = MarkerArray()

        marker = Marker()
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)

        self.markers_pub.publish(markers)

    def loadPath(self, marker):
        filename = input("Path input filename? ")

        pathCSV = open(root_dir + "/pure_pursuit/paths/" + filename, "r")

        reader = csv.reader(pathCSV, delimiter=',')

        self.path = []

        for row in reader:
            self.path.append({
                "x": float(row[0]),
                "y": float(row[1]),
                "rot": float(row[2]),
                "speed": float(row[3]),
                "lookahead": float(row[4])
            })
            
        pathCSV.close()

    def savePath(self, marker):
        filename = input("Path output filename? ")

        pathCSV = open(root_dir + "/pure_pursuit/paths/" + filename, "w")

        writer = csv.writer(pathCSV, delimiter=',')

        for waypoint in self.path:
            writer.writerow([waypoint["x"], waypoint["y"], waypoint["rot"], waypoint["speed"], waypoint['lookahead']])

        pathCSV.close()

if __name__ == '__main__':
    rclpy.init()
    print("InteractiveNode Initialized")
    interactive_marker_node = InteractiveMarkerNode()

    server = InteractiveMarkerServer(interactive_marker_node, "i_marker_server")

    menu_handler.insert("New Marker", callback=interactive_marker_node.newMarker)
    menu_handler.insert("Marker Details", callback=interactive_marker_node.details)
    menu_handler.insert("Delete Marker", callback=interactive_marker_node.deleteMarker)
    menu_handler.insert("Load Markers", callback=interactive_marker_node.loadMarkers)
    menu_handler.insert("Save Markers", callback=interactive_marker_node.saveMarkers)

    alignment_menu_handle = menu_handler.insert("Alignment")
    menu_handler.insert("Align X", parent=alignment_menu_handle, callback=interactive_marker_node.alignX)
    menu_handler.insert("Align Y", parent=alignment_menu_handle, callback=interactive_marker_node.alignY)
    menu_handler.insert("Rotation Alignment", parent=alignment_menu_handle, callback=interactive_marker_node.rotationAlignment)

    waypoint_menu_handle = menu_handler.insert("Waypoints")
    menu_handler.insert("Set Speed", parent=waypoint_menu_handle, callback=interactive_marker_node.setSpeed)
    menu_handler.insert("Set Lookahead", parent=waypoint_menu_handle, callback=interactive_marker_node.setLookahead)

    path_menu_handle = menu_handler.insert("Path")
    # menu_handler.insert("Render Path", parent=path_menu_handle, callback=interactive_marker_node.renderPath) # rendering path on each change now
    # menu_handler.insert("Delete Path", parent=path_menu_handle, callback=interactive_marker_node.deletePath)
    # menu_handler.insert("Load Path", parent=path_menu_handle, callback=interactive_marker_node.loadPath)
    menu_handler.insert("Save Path", parent=path_menu_handle, callback=interactive_marker_node.savePath)

    interactive_marker_node.newMarker()

    rclpy.spin(interactive_marker_node)

    interactive_marker_node.destroy_node()
    rclpy.shutdown()