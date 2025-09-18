import numpy as np
from quaternion_math import quaternion_to_rotm
def state_mux(p, v, q, w):
    state = np.zeros((13,))
    state[0:3] = p
    state[3:6] = v
    state[6:10] = q
    state[10:13] = w
    return state

def state_demux(s, tf_req=False):
    p = np.zeros((3,))
    v = np.zeros((3,))
    q = np.zeros((4,))
    w = np.zeros((3,))

    p = s[0:3]
    v_temp = s[3:6]
    q = s[6:10]
    w = s[10:13]

    if tf_req:
        rotm = quaternion_to_rotm(q)
        v_world = rotm @ v_temp
    else:
        v_world = v_temp
    return p, v, q, w