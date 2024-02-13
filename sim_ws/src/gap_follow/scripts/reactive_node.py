import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

float32 = np.float32

# car dimensions
CAR_WIDTH  : float32 = 0.29 # meters
CAR_LENGTH : float32 = 0.45 # meters

# topics
SCAN_TOPIC  : str = "/scan"
DRIVE_TOPIC : str = "/drive"

# refresh frequencies
SCAN_REFRESH = 10
DRIVE_REFRESH = 10

# preprocessing range constants
RANGE_SIZE : int = 1080
WIND_SIZE  : int = 5

# distance processing
DIST_THRESH : float32 = 3.0

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, SCAN_TOPIC, self.lidar_callback, SCAN_REFRESH
        )

        # publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, DRIVE_TOPIC, DRIVE_REFRESH
        )

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = ranges.copy()

        wind_rad  : int = WIND_SIZE // 2
        proc_size : int = RANGE_SIZE - 2 * wind_rad

        for i in range(proc_size):
            sum : float32 = 0.0
            # elems = []

            for j in range(WIND_SIZE):
                sum += ranges[i + j]
                # elems.append(ranges[i + j])

            # print(f"{i}: {elems} | {sum} :: {sum / float(WIND_SIZE)}")
            sum = np.round(sum / float(WIND_SIZE), 3)

            if sum > DIST_THRESH:
                sum = 0.0  # invalidate distance
            
            proc_ranges[i + wind_rad] = sum

        # duplicate average values into edge of ranges array
        for i in range(wind_rad):
            proc_ranges[i] = proc_ranges[wind_rad]
            proc_ranges[RANGE_SIZE - i - 1] = proc_ranges[RANGE_SIZE - wind_rad - 1]

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        return None
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        return None

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 

        #Find the best point in the gap 

        #Publish Drive message


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
