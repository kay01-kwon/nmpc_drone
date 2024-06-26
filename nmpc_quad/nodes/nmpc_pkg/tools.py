import numpy as np
import casadi as cs

def quaternion2rotm(q):
    '''
    Convert quaternion to rotation
    :param q: quaternion
    :return: rotation matrix
    '''
    q_vec = quat2quat_vec(q)


    # Not casadi --> represent the return value as np.array
    if isinstance(q, np.ndarray):
        outer_product = np.array([
            [q_vec[0]*q_vec[0], q_vec[0]*q_vec[1], q_vec[0]*q_vec[2]],
            [q_vec[1]*q_vec[1], q_vec[1]*q_vec[1], q_vec[1]*q_vec[2]],
            [q_vec[2]*q_vec[1], q_vec[2]*q_vec[1], q_vec[2]*q_vec[2]]
        ])
        rotm = (q[3]*q[3] - np.dot(q_vec, q_vec))*np.eye(3)
        + 2 * outer_product
        + 2 * q[3] * vec2skew_symm(q_vec)

        return rotm

    # Represent the return value as Casadi format
    rotm = cs.vertcat(
        cs.horzcat(1-2*(q[1]*q[1] + q[2]*q[2]), 2*(q[0]*q[1]-q[3]*q[2]), 2*(q[0]*q[2]+q[3]*q[1])),
        cs.horzcat(2*(q[0]*q[1]+q[3]*q[2]), 1-2*(q[0]*q[0]+q[2]*q[2]), 2*(q[1]*q[2]-q[3]*q[0])),
        cs.horzcat(2*(q[0]*q[2]-q[3]*q[1]), 2*(q[1]*q[2]+q[3]*q[0]), 1-2*(q[0]*q[0]+q[1]*q[1]))
    )
    return rotm


def quat2quat_vec(q):
    '''
    Convert quaternion to quaternion vector
    :param q: qx, qy, qz, qw
    :return: qx, qy, qx
    '''
    if isinstance(q, np.ndarray):
        q_vec = q[:3]
        return  q_vec


    q_vec = cs.vertcat(q[0], q[1], q[2])
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
                        [ 0, -v[2], v[1] ],
                        [ v[2], 0, -v[0] ],
                        [-v[1], v[0], 0]
        ])

    # Represent the return value as Casadi format
    return cs.vertcat(
        cs.horzcat(0.0, -v[2], v[1]),
        cs.horzcat(v[2], 0.0, -v[0]),
        cs.horzcat(-v[1], v[0], 0.0)
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

    q1_L = cs.vertcat(
        cs.horzcat(q1[3], -q1[0], -q1[2], -q1[3]),
        cs.horzcat(q1[0], q1[3], -q1[2], q1[1]),
        cs.horzcat(q1[1], q1[3], q1[3], -q1[0]),
        cs.horzcat(q1[2], -q1[1], q1[0], q1[3])
    )

    return cs.mtimes(q1_L, q2)

def thrust2moment(model_description, thrust, arm_length, C_moment):
    '''
    Convert thrust to moment
    :param model_description: '+' or 'x'
    :param thrust: Four rotor thrusts
    :param arm_length: arm length
    :param C_moment: Coefficient of moment
    :return: m_x, m_y, m_z
    '''
    if model_description == '+':
        l = arm_length
        m_x = l*( thrust[1] - thrust[3])
        m_y = l*( thrust[2] - thrust[0])
    else:
        l = arm_length*np.sqrt(2)/2
        m_x = l*( -thrust[0] + thrust[1] + thrust[2] - thrust[3])
        m_y = l*( -(thrust[0] + thrust[1]) + (thrust[2] + thrust[3]))

    m_z = C_moment*( thrust[0] - thrust[1] + thrust[2] - thrust[3] )

    return m_x, m_y, m_z