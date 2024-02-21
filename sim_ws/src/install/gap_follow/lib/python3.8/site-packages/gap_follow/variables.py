import numpy as np
import gap_follow.helper as hp

float32 = np.float32

# refresh frequencies
SCAN_REFRESH = 10
DRIVE_REFRESH = 10

# scan preprocessing constants
RANGE_SIZE : int = 1080
WIND_SIZE  : int = 5
FOV : float = hp.deg_to_rad(270.0)

# distance processing
DIST_THRESH  : float32 = 6.00 # meters
DISP_EPSILON : float32 = 0.20 # meters

# car dimensions
CAR_WIDTH : float32 = 0.40 # meters

# distances
LEFT : int = hp.rad_to_idx(hp.deg_to_rad(90.0), FOV, RANGE_SIZE)
RIGHT : int = hp.rad_to_idx(hp.deg_to_rad(-90.0), FOV, RANGE_SIZE)