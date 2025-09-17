import numpy as np
from math_tools import quaternion_math
class RcController():

    def __init__(self, GainParam, DynParam):

        # Translational control gain
        self.Kp_trans_ = GainParam['Kp_trans']
        self.Kd_trans_ = GainParam['Kd_trans']

        # Orientational control gain
        self.Kp_ori_ = GainParam['Kp_ori']
        self.Kd_ori_ = GainParam['Kd_ori']

        self.m_ = DynParam['m']
        self.J_ = DynParam['J']

        self.g_vec = np.array([0.0, 0.0, -9.81])

        self.axis_des_ = np.zeros((3,))

        self.u_ = np.zeros((3,))

    def set_ref_state(self, ref, state, tau):
        '''
        Set the reference and state of the controller
        :param ref: ax_des, ay_des, z_des, dpsi_dt_des
        :param state: p, v, q, w
        :param tau: orientational disturbance
        '''
        ax_des = ref[0]
        ay_des = ref[1]
        z_des = ref[2]
        dpsi_dt_des = ref[3]
        a_des = np.array([ax_des, ay_des, 0])
        p_des = np.array([0, 0, z_des])
        w_des = np.array([0, 0, dpsi_dt_des])

        p = np.array([0, 0, state[2]])
        v = state[3:6]
        q = state[6:10]
        w = state[10:13]

        R = (quaternion_math.
                quaternion_to_rotm(q))

        p_err = p - p_des
        v_err = R @ v

        f_des = (self.m_ * a_des
                 - self.Kp_trans_ @ p_err
                 - self.Kd_trans_ @ v_err
                 -self.m_ * self.g_vec)

        self.u_[0] = np.sqrt(f_des.transpose() @ f_des)

        r = f_des/self.u_[0]

        rz = np.sqrt(1 - (r[0]**2 + r[1]**2))

        cos_half_phi = np.sqrt( (1 + rz) / 2.0 )
        sin_half_phi = np.sqrt( (1 - rz) / 2.0)
        sin_phi = np.sqrt( 1 - rz**2)

        if np.abs(sin_phi) > 1e-30:
            self.axis_des_[0] = -1 / sin_phi * r[1]
            self.axis_des_[1] = 1 / sin_phi * r[0]
        else:
            self.axis_des_[0] = 0
            self.axis_des_[1] = 0

        q_rp_des = np.array((4,))
        q_rp_des[0] = cos_half_phi
        q_rp_des[1:] = self.axis_des_ * sin_half_phi

        half_psi = np.arctan2(q[3], q[0])
        cos_half_psi = np.cos(half_psi)
        sin_half_phi = np.sin(half_psi)
        q_yaw = np.array([cos_half_psi, 0, 0, sin_half_phi])

        q_des = quaternion_math.otimes(q_rp_des, q_yaw)


        q_err = quaternion_math.otimes(q_des, q)
        q_err_vec = self._signum(q_err[0])*q_err[1:]

        R_des = quaternion_math.quaternion_to_rotm(q_des)

        w_err = w - R.transpose() @ R_des @ w_des

        M_pd = (-self.Kp_ori_ @ q_err_vec
             - self.Kd_ori_ @ w_err
             + np.cross(w, self.J_ @ w))

        self.u_[1:] = M_pd - tau

    def get_control_input(self):
        return self.u_

    def _signum(self,x):
        return 1 if x >= 0 else -1



