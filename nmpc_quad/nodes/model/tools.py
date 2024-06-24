import numpy as np
import casadi as cs
g = 9.81

def quaternion2rotm(q):

    q_vec = quat2quat_vec(q)

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


def quat2quat_vec(q):
    q_vec = q[:3]
    return q_vec

def vec2skew_symm(v):

    if isinstance(v, np.ndarray):
        return np.array([
                        [ 0, -v[2], v[1] ],
                        [ v[2], 0, -v[0] ],
                        [-v[1], v[0], 0] ])

    return cs.vertcat(
        cs.horzcat(0, -v[2], v[1]),
        cs.horzcat(v[2], 0, -v[0]),
        cs.horzcat([-v[1], v[0], 0])
    )