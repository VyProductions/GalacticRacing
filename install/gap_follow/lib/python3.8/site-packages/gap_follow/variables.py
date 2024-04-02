import numpy as np
import gap_follow.helper as hp

float32 = np.float32

# refresh frequencies
SCAN_REFRESH = 10
DRIVE_REFRESH = 10

# preprocessing constants
WIND_SIZE     : int = 5   # kerneling size for sliding average window
SAMPLE_RADIUS : int = 10  # for attempting to isolate tiny gaps more effectively

# distance processing
DIST_THRESH  : float32 = 6.00  # meter cutoff for scan processing
DISP_EPSILON : float32 = 0.12  # minimum meter difference to identify disparities

# car dimensions
CAR_WIDTH : float32 = 0.40 # car width in meters

# lidar properties
RANGE_SIZE : int = 1080  # 1080 samples populating the scan array
FOV : float32 = hp.deg_to_rad(270.0)  # 135 degrees on either side of forward
LEFT  : int = hp.rad_to_idx(hp.deg_to_rad(55.0), FOV, RANGE_SIZE)   # index of left-most scan used to gap follow
RIGHT : int = hp.rad_to_idx(hp.deg_to_rad(-55.0), FOV, RANGE_SIZE)  # index of right-most scan used to gap follow
PEEK_LEFT : int = hp.rad_to_idx(hp.deg_to_rad(90.0), FOV, RANGE_SIZE)
PEEK_RIGHT : int = hp.rad_to_idx(hp.deg_to_rad(-90.0), FOV, RANGE_SIZE)

# modifier constants
BUBBLE_MULT : float32 = 1.2
