#!/usr/bin/env python3

import numpy as np

float32 = np.float32

def deg_to_rad(deg : float32) -> float32:
    return (deg * np.pi) / 180.0

# preprocessing range constants
RANGE_SIZE : int = 10
WIND_SIZE  : int = 5
FOV : float32 = deg_to_rad(270.0)

# distance processing
DIST_THRESH : float32 = 5.0
CAR_WIDTH : float32 = 0.29

def preprocess_lidar(ranges):
    """ Preprocess the LiDAR scan array. Expert implementation includes:
        1.Setting each value to the mean over some window
        2.Rejecting high values (eg. > 3m)
    """
    proc_ranges = [None] * RANGE_SIZE

    wind_rad  : int = WIND_SIZE // 2
    proc_size : int = RANGE_SIZE - 2 * wind_rad

    for i in range(proc_size):
        sum : float32 = 0.0
        elems = []

        for j in range(WIND_SIZE):
            sum += ranges[i + j]
            elems.append(ranges[i + j])

        print(f"{i}: {elems} | {sum} :: {sum / float(WIND_SIZE)}")
        sum = np.round(sum / float(WIND_SIZE), 3)

        if sum > DIST_THRESH:
            sum = 0.0  # invalidate distance
        
        proc_ranges[i + wind_rad] = sum

    # duplicate average values into edge of ranges array
    for i in range(wind_rad):
        proc_ranges[i] = proc_ranges[wind_rad]
        proc_ranges[RANGE_SIZE - i - 1] = proc_ranges[RANGE_SIZE - wind_rad - 1]

    return proc_ranges

ranges_list = [5.6, 9.2, 3.8, 1.4, 0.7, 8.3, 2.1, 6.5, 4.9, 7.0]
proc_list   = preprocess_lidar(ranges_list)

print(ranges_list)
print(proc_list)

def find_max_gap(free_space_ranges):
    """ Return the start index & end index of the max gap in free_space_ranges
    """
    f_idx  : int = -1
    l_idx  : int = -1
    first  : int = -1
    last   : int = -1
    in_gap : bool = False

    for index in range(RANGE_SIZE):
        # start of non-zero distance gap detected
        if not in_gap and free_space_ranges[index] != 0.0:
            in_gap = True
            f_idx = index
        
        # end of non-zero distance gap detected
        elif in_gap and free_space_ranges[index] == 0.0:
            in_gap = False
            l_idx = index

            # update max gap boundaries if applicable
            if l_idx - f_idx > last - first:
                first = f_idx
                last = l_idx

    if first == -1 and in_gap:
        first = 0
        last = RANGE_SIZE - 1
        return first, last-1
    elif first == -1:
        print("Could not locate a gap.")
        return -1, -1
    
    return first, last - 1

print(f"Max gap boundary: {find_max_gap(proc_list)}")

def area_of_collision(close_idx, proc_ranges):
    close_angle = close_idx * FOV / RANGE_SIZE - FOV / 2.0
    theta_offs = np.arctan((CAR_WIDTH / 2.0) / proc_ranges[close_idx])
    close_left = close_angle + theta_offs
    close_right = close_angle - theta_offs

    # clamp bounds of bubble to lidar array boundary
    if close_left > FOV / 2.0:
        close_left = FOV / 2.0
    if close_right < -FOV / 2.0:
        close_right = -FOV / 2.0

    # get indices from angles
    left_idx : int = int(np.round((close_left + FOV / 2.0) * (RANGE_SIZE / FOV)))
    right_idx : int = int(np.round((close_right + FOV / 2.0) * (RANGE_SIZE / FOV)))

    # wipe data in bubble
    for i in range(left_idx, right_idx + 1):
        proc_ranges[i] = 0.0

print(9 * FOV / (RANGE_SIZE - 1) - FOV / 2.0)


for i in range(10, 2, -1):
    print(i)