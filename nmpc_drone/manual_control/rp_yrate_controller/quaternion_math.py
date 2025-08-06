import numpy as np


def otimes(q1, q2):
    q1_w = q1[0]
    q1_x = q1[1]
    q1_y = q1[2]
    q1_z = q1[3]

    q1_mat = np.array([
        [q1_w, -q1_x, -q1_y, -q1_z],
        [q1_x, q1_w, -q1_z, q1_y],
        [q1_y, q1_z, q1_w, -q1_x],
        [q1_z, -q1_y, q1_x, q1_w]
    ])

    result = q1_mat@q2
    return result

def conjugate(q):
    q_res = np.array([q[0], -q[1], -q[2], -q[3]])
    return q_res
def quaternion_to_rotm(q):
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    rotm = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qy * qx + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qz * qx - qw * qy), 2 * (qz * qy + qw * qx), 1 - 2 * (qx * qx + qy * qy)]
    ])

    return rotm

def skew_symm(v):
    vx = v[0]
    vy = v[1]
    vz = v[2]
    return np.array([
        [0, -vz, vy],
        [vz, 0, -vx],
        [-vy, vx, 0]
    ])