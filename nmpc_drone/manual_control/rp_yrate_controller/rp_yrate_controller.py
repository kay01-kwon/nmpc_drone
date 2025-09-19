import numpy as np
from utils import quaternion_math
class RpYrateController:
    def __init__(self, Kp, Kd, J):
        self.Kp = Kp
        self.Kd = Kd
        self.J = J
        self.axis_d = np.zeros((3,))
        self.q_rp_d = np.zeros((4,))
        self.q_d = np.zeros((4,))
        self.q_state = np.zeros((4,))
        self.w_state = np.zeros((3,))
        self.omega_d = np.zeros((3,))

    def set_rp_yrate(self, r, y_rate, q_state, w_state):

        rz = np.sqrt(1 - (r[0]**2 + r[1]**2))
        cos_half_phi = np.sqrt( (1 + rz) / 2)
        sin_half_phi = np.sqrt( (1 - rz) / 2)
        sin_phi = np.sqrt( 1 - rz**2)

        if np.abs(sin_phi) > 1e-10:
            self.axis_d[0] = -1 / sin_phi * r[1]
            self.axis_d[1] = 1 / sin_phi * r[0]
        else:
            self.axis_d[0] = 0
            self.axis_d[1] = 0

        self.omega_d[2] = y_rate

        self.q_rp_d[0] = cos_half_phi
        self.q_rp_d[1:] = self.axis_d * sin_half_phi

        qw = q_state[0]
        qz = q_state[3]

        self.q_state = q_state
        self.w_state = w_state

        half_psi = np.arctan2(q_state[3], q_state[0])
        cos_half_psi = np.cos(half_psi)
        sin_half_psi = np.sin(half_psi)
        q_yaw = np.array([cos_half_psi, 0, 0, sin_half_psi])

        self.q_d = quaternion_math.otimes(self.q_rp_d, q_yaw)

    def compute_moment(self):

        q_e = quaternion_math.otimes(quaternion_math.conjugate(self.q_d),
                                    self.q_state)
        q_e_vec = self.signum(q_e[0])*q_e[1:]

        R = quaternion_math.quaternion_to_rotm(self.q_state)
        Rd = quaternion_math.quaternion_to_rotm(self.q_d)
        w_e = self.w_state - R.transpose() @ Rd @ self.omega_d

        M = (-self.Kp @ q_e_vec - self.Kd @ w_e
             + quaternion_math.skew_symm(self.w_state) @ self.J @ self.w_state)

        return M
    def signum(self, x):
        return 1 if x > 0 else -1