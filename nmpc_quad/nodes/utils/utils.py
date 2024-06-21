import numpy as np

g = 9.81

def rotm_from_quaternion(quat):
    rotm = np.eye(3)
    return rotm