import numpy as np

float32 = np.float32

def deg_to_rad(deg : float32) -> float32:
    return (deg * np.pi) / 180.0

def rad_to_deg(rad : float32) -> float32:
    return (rad * 180.0) / np.pi

def idx_to_rad(idx : int, FOV : float32, RANGE_SIZE : int) -> float32:
    return idx * FOV / (RANGE_SIZE - 1) - FOV / 2.0

def rad_to_idx(rad : float32, FOV : float32, RANGE_SIZE : int) -> int:
    return int(np.round((rad + FOV / 2.0) * (RANGE_SIZE / FOV)))

def sign(value):
    if value < 0:
        return -1
    else:
        return 1

def get_progress(start, finish, value):
    if start == finish:
        return 1.0
    elif sign(finish - start) != sign(finish - value):
        return 1.0
    else:
        return (value - start) / (finish - start)
