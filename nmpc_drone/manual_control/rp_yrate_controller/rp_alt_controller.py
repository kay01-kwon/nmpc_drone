import numpy as np
from manual_control.rp_yrate_controller import quaternion_math
class RpyzController():
    def __init__(self, GainParam, DynParam):
        self.Kp_trans = GainParam['Kp_trans']
        self.Kv_trans = GainParam['Kv_trans']
        self.Kp_ori = GainParam['Kp_ori']
        self.Kd_ori = GainParam['Kd_ori']
        self.m = DynParam['m']
        self.J = DynParam['J']
        self.g_vec = np.array([0,0,-9.81])
        self.axis_d = np.zeros((3,))
        self.q_rp_d = np.zeros((4,))
        self.q_d = np.zeros((4,))
        self.q_state = np.zeros((4,))
        self.w_state = np.zeros((3,))
        self.omega_d = np.zeros((3,))
        self.u = np.zeros((4,))

    def set_rp_yrate(self, a_ref, p_ref, psidot, p_state, v_state, q_state, w_state):

        f_des = (self.m * a_ref
                 - self.Kp_trans @ (p_state - p_ref)
                 - self.Kv_trans @ v_state
                 - self.m * self.g_vec)
        # print(f_des)

        self.u[0] = np.sqrt(f_des.transpose() @ f_des)

        r = f_des/self.u[0]

        rz = np.sqrt(1 - (r[0]**2 + r[1]**2))

        cos_half_phi = np.sqrt( (1 + rz) / 2)
        sin_half_phi = np.sqrt( (1 - rz) / 2)
        sin_phi = np.sqrt( 1 - rz**2)

        if np.abs(sin_phi) > 1e-50:
            self.axis_d[0] = -1 / sin_phi * r[1]
            self.axis_d[1] = 1 / sin_phi * r[0]
        else:
            self.axis_d[0] = 0
            self.axis_d[1] = 0

        self.omega_d[2] = psidot

        self.q_rp_d[0] = cos_half_phi
        self.q_rp_d[1:] = self.axis_d * sin_half_phi

        self.q_state = q_state
        self.w_state = w_state

        half_psi = np.arctan2(q_state[3], q_state[0])
        cos_half_psi = np.cos(half_psi)
        sin_half_psi = np.sin(half_psi)
        q_yaw = np.array([cos_half_psi, 0, 0, sin_half_psi])

        self.q_d = quaternion_math.otimes(self.q_rp_d, q_yaw)
        # self.q_d = self.q_rp_d


    def compute_moment(self, tau):

        q_e = quaternion_math.otimes(quaternion_math.conjugate(self.q_d),
                                    self.q_state)
        q_e_vec = self.signum(q_e[0])*q_e[1:]

        R = quaternion_math.quaternion_to_rotm(self.q_state)
        Rd = quaternion_math.quaternion_to_rotm(self.q_d)
        w_e = self.w_state - R.transpose() @ Rd @ self.omega_d
        M = (-self.Kp_ori @ q_e_vec - self.Kd_ori @ w_e
             + quaternion_math.skew_symm(self.w_state) @ self.J @ self.w_state)
        # M = (-self.Kp_ori @ q_e_vec - self.Kd_ori @ w_e)

        self.u[1:] = M - tau

        return self.u
    def signum(self, x):
        return 1 if x > 0 else -1