import numpy as np
import casadi as cs
g = 9.81

def quaternion2rotm(q):

    q_vec = quat2quat_vec(q)


    # Not casadi --> represent the return value as np.array
    if isinstance(q, np.ndarray):
        outer_product = np.array([
            [q_vec[0]*q_vec[0], q_vec[0]*q_vec[1], q_vec[0]*q_vec[2]],
            [q_vec[1]*q_vec[1], q_vec[1]*q_vec[1], q_vec[1]*q_vec[2]],
            [q_vec[2]*q_vec[1], q_vec[2]*q_vec[1], q_vec[2]*q_vec[2]]
        ])
        rotm = (q[3]*q[3] - sum(q_vec[:]*q_vec[:])*np.eye(3)
        + 2 * outer_product
        + 2 * q[3] * vec2skew_symm(q_vec)

        return rotm

    # Represent the return value as Casadi format
    eye_mat = cs.vertcat(
        cs.horzcat(1.0, 0.0, 0.0),
        cs.horzcat(0.0, 1.0, 0.0),
        cs.horzcat(0.0, 0.0, 1.0)
    )

    outer_product = cs.vertcat(
        cs.horzcat(q_vec[0]*q_vec[0], q_vec[0]*q_vec[1], q_vec[0]*q_vec[2]),
        cs.horzcat(q_vec[1]*q_vec[1], q_vec[1]*q_vec[1], q_vec[1]*q_vec[2]),
        cs.horzcat(q_vec[2]*q_vec[1], q_vec[2]*q_vec[1], q_vec[2]*q_vec[2])
    )

    rotm = (q[3]*q[3] - sum(q_vec[:]*q_vec[:])*eye_mat
    + 2 * outer_product
    + q[3] * vec2skew_symm(q_vec)

    return rotm


def quat2quat_vec(q):
    q_vec = q[:3]
    return q_vec

def vec2skew_symm(v):

    # Not casadi --> represent the return value as np.array
    if isinstance(v, np.ndarray):
        return np.array([
                        [ 0, -v[2], v[1] ],
                        [ v[2], 0, -v[0] ],
                        [-v[1], v[0], 0] ])

    # Represent the return value as Casadi format
    return cs.vertcat(
        cs.horzcat(0, -v[2], v[1]),
        cs.horzcat(v[2], 0, -v[0]),
        cs.horzcat([-v[1], v[0], 0])
    )

def otimes(q1,q2):

    # Not Casadi form --> return np.array
    if isinstance(q1,np.ndarray):
        q1_L = np.array([
            [q1[3], -q1[0], -q1[2], -q1[3]],
            [q1[0], q1[3], -q1[2], q1[1]],
            [q1[1], q1[3], q1[3], -q1[0]],
            [q1[2], -q1[1], q1[0], q1[3]]
        ])
        return np.matmul(q1_L, q2)

    q1_L = cs.vertcat(
        cs.horzcat(q1[3], -q1[0], -q1[2], -q1[3]),
        cs.horzcat(q1[0], q1[3], -q1[2], q1[1]),
        cs.horzcat(q1[1], q1[3], q1[3], -q1[0]),
        cs.horzcat(q1[2], -q1[1], q1[0], q1[3])
    )

    return cs.mtimes(q1_L, q2)
