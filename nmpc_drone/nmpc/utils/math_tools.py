import numpy as np
import casadi as cs

def quaternion2rotm(q):
    '''
    Convert quaternion to rotation
    :param q: quaternion
    :return: rotation matrix
    '''
    # Not casadi --> represent the return value as np.array
    if isinstance(q, np.ndarray):
        qw = q[0]
        qx = q[1]
        qy = q[2]
        qz = q[3]

        rotm = np.array([
            [1-2*(qy*qy + qz*qz), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)],
            [2*(qy*qx+qw*qz), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx)],
            [2*(qz*qx-qw*qy), 2*(qz*qy+qw*qx), 1-2*(qx*qx+qy*qy)]
        ])
        return rotm

    # Casadi format
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    # Represent the return value as Casadi format
    rotm = cs.vertcat(
        cs.horzcat(1-2*(qy*qy + qz*qz), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)),
        cs.horzcat(2*(qy*qx+qw*qz), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx)),
        cs.horzcat(2*(qz*qx-qw*qy), 2*(qz*qy+qw*qx), 1-2*(qx*qx+qy*qy))
    )
    return rotm


def quat2quat_vec(q):
    '''
    Convert quaternion to quaternion vector
    :param q: qx, qy, qz, qw
    :return: qx, qy, qx
    '''
    if isinstance(q, np.ndarray):
        q_vec = q[1:4]
        return  q_vec

    qx = q[1]
    qy = q[2]
    qz = q[3]

    q_vec = cs.vertcat(qx, qy, qz)
    return q_vec

def vec2skew_symm(v):
    '''
    Convert vector to skew symmetric matrix
    :param v: vx, vy, vz
    :return: skew symmetric matrix
    '''

    # Not casadi --> represent the return value as np.array
    if isinstance(v, np.ndarray):
        return np.array([
                        [ 0,    -v[2],  v[1] ],
                        [ v[2],     0, -v[0] ],
                        [-v[1],  v[0],    0  ]
        ])

    vx = v[0]
    vy = v[1]
    vz = v[2]

    # Represent the return value as Casadi format
    return cs.vertcat(
        cs.horzcat(0.0, -vz, vy),
        cs.horzcat(vz, 0.0, -vx),
        cs.horzcat(-vy, vx, 0.0)
    )

def otimes(q1,q2):
    '''
    Calculate multiplication of two quaternion
    :param q1: Left quaternion
    :param q2: Right quaternion
    :return: otimes
    '''
    # Not Casadi form --> return np.array
    if isinstance(q1,np.ndarray):

        q1_L = np.array([
            [q1[3], -q1[0], -q1[2], -q1[3]],
            [q1[0], q1[3], -q1[2], q1[1]],
            [q1[1], q1[3], q1[3], -q1[0]],
            [q1[2], -q1[1], q1[0], q1[3]]
        ])
        return np.matmul(q1_L, q2)

    den1 = cs.sqrt(q1[0]**2 + q1[1]**2 + q1[2]**2 + q1[3]**2)
    den2 = cs.sqrt(q2[0]**2 + q2[1]**2 + q2[2]**2 + q2[3]**2)

    q1_w = q1[0]
    q1_x = q1[1]
    q1_y = q1[2]
    q1_z = q1[3]

    q1_L = cs.vertcat(
        cs.horzcat(q1_w, -q1_x, -q1_y, -q1_z),
        cs.horzcat(q1_x, q1_w, -q1_z, q1_y),
        cs.horzcat(q1_y, q1_z, q1_w, -q1_x),
        cs.horzcat(q1_z, -q1_y, q1_x, q1_w)
    )

    return cs.mtimes(q1_L, q2)