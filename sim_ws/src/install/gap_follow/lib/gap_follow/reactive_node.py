#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

float32 = np.float32

def deg_to_rad(deg : float32) -> float32:
    return (deg * np.pi) / 180.0

def rad_to_deg(rad : float32) -> float32:
    return (rad * 180.0) / np.pi

# refresh frequencies
SCAN_REFRESH = 10
DRIVE_REFRESH = 10

# scan preprocessing constants
RANGE_SIZE : int = 1080
WIND_SIZE  : int = 5
FOV : float = deg_to_rad(270.0)

# distance processing
DIST_THRESH  : float32 = 2.40 # meters
DISP_EPSILON : float32 = 0.20 # meters

# car dimensions
CAR_WIDTH  : float32 = 0.45 # meters
CAR_LENGTH : float32 = 0.45 # meters

def idx_to_rad(idx : int) -> float32:
    return idx * FOV / (RANGE_SIZE - 1) - FOV / 2.0

def rad_to_idx(rad : float32) -> int:
    return int(np.round((rad + FOV / 2.0) * (RANGE_SIZE / FOV)))

LEFT : int = rad_to_idx(deg_to_rad(90.0))
RIGHT : int = rad_to_idx(deg_to_rad(-90.0))

class ReactiveFollowGap(Node):
    """ 
    Implement Follow the Gap on the car
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, lidarscan_topic, self.lidar_callback, SCAN_REFRESH
        )

        # publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, DRIVE_REFRESH
        )

        self.steering_angle = 0.0 # rads relative to forward (left: > 0.0, right: < 0.0)

        self.kp = 1.0
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = [None] * RANGE_SIZE

        wind_rad  : int = WIND_SIZE // 2
        proc_size : int = RANGE_SIZE - 2 * wind_rad

        for i in range(proc_size):
            sum : float32 = 0.0

            for j in range(WIND_SIZE):
                if ranges[i + j] < 0.0:
                    ranges[i + j] = 0.0

                sum += ranges[i + j]

            sum = np.round(sum / float(WIND_SIZE), 2)

            if sum > DIST_THRESH:
                sum = float("inf")  # too far away; make it infinity
            
            proc_ranges[i + wind_rad] = sum

        # duplicate average values into edge of ranges array
        for i in range(wind_rad):
            proc_ranges[i] = proc_ranges[wind_rad]
            proc_ranges[RANGE_SIZE - i - 1] = proc_ranges[RANGE_SIZE - wind_rad - 1]

        return proc_ranges

    def disparity_extender(self, proc_ranges):
        # step 2: find all disparities
        i : int = RIGHT
        while i <= LEFT:
            diff :float32 = proc_ranges[i] - proc_ranges[i + 1]
            # step 3: extend disparities
            if abs(diff) > DISP_EPSILON:
                if diff < 0: # left point is farther away
                    theta : float32 = np.arcsin(CAR_WIDTH / (2 * proc_ranges[i]))
                    targ_idx : int = rad_to_idx(idx_to_rad(i) + theta)

                    # extend disparities left
                    for j in range(i + 1, targ_idx + 1):
                        if proc_ranges[j] > proc_ranges[i]:
                            proc_ranges[j] = proc_ranges[i]
                    
                    i = targ_idx + 1
                else: # right point is farther away
                    theta : float32 = np.arcsin(CAR_WIDTH / (2 * proc_ranges[i + 1]))
                    targ_idx : int = rad_to_idx(idx_to_rad(i + 1) - theta)

                    # extend disparities right
                    for j in range(targ_idx, i + 1):
                        if proc_ranges[j] > proc_ranges[i + 1]:
                            proc_ranges[j] = proc_ranges[i + 1]
                    i += 1
            else:
                i += 1
        
        return proc_ranges

    def max_gap(self, disp_ranges):
        """
        Return the best index as the midpoint of the largest gap in disp_ranges
        """
        thresh = DIST_THRESH

        f_idx  : int = -1
        l_idx  : int = -1
        first  : int = -1
        last   : int = -1
        in_gap : bool = False

        for index in range(RIGHT, LEFT + 1):
            # start of non-zero distance gap detected
            if not in_gap and disp_ranges[index] > thresh:
                in_gap = True
                f_idx = index
            
            # end of non-zero distance gap detected
            elif in_gap and disp_ranges[index] <= thresh:
                in_gap = False
                l_idx = index

                # update max gap boundaries if applicable
                if l_idx - f_idx > last - first:
                    first = f_idx
                    last = l_idx

        if first == -1 and in_gap:
            first = RIGHT
            last = LEFT + 1
        elif first == -1:
            print("Could not find a gap.")
    
        PROP : float32 = 0.5

        if abs(540 - first) < abs(540 - last):
            return first + np.round(PROP * (last - first))
        else:
            return last - np.round(PROP * (last - first))

        # return (first + last) // 2

        # longest_idx : int = last

        # for i in range(last - 1, first - 1):
        #     if disp_ranges[i] > disp_ranges[longest_idx]:
        #         longest_idx = i

        # return longest_idx

    def lidar_callback(self, data):
        """
        Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges

        # preprocess range data
        # proc_ranges = self.preprocess_lidar(ranges)

        # filter range data using disparity extender algorithm
        disp_ranges = self.disparity_extender(ranges)

        # get best index by finding the largest gap and aiming towards its center
        best_idx = self.max_gap(disp_ranges)

        # Determine steering angle from best point, if valid
        angle : float32

        if best_idx != -1:
            angle = idx_to_rad(best_idx)
        else:
            angle = 0.0

        # Clamp angle between min and max steering angle
        if angle < deg_to_rad(-20.0):
            angle = deg_to_rad(-20.0)
        elif angle > deg_to_rad(20.0):
            angle = deg_to_rad(20.0)

        # print(f"Steering to angle: {rad_to_deg(angle)}")
        self.steering_angle = self.steering_angle + 0.9 * (angle - self.steering_angle)
        # self.steering_angle = 0.0

        # Adjust velocity based on distance
        velocity : float32
        if ranges[540] < 0.75:
            velocity = 0.0
        else:
            velocity = ranges[540] * 0.8

        #Publish Drive message
        msg = AckermannDriveStamped()
        msg.drive.speed = velocity
        msg.drive.steering_angle = self.steering_angle
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    print("Gap Follow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
