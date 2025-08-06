import numpy as np

class RcConverter:
    def __init__(self, f_max, phi_max, psidot_max):
        # Maximum collective thrust, phi max (deg), psidot_max (deg/s)
        # Initialize rc input as zeros

        self.throttle_in = 0.0
        # r[0]: rx
        # r[1]: ry
        self.r = np.zeros((2,))
        self.yaw_rate = 0.0

        self.rc_in_mid = 1500
        self.rc_in_delta = 512
        self.rc_in_min = self.rc_in_mid - self.rc_in_delta
        self.rc_in_max = self.rc_in_mid + self.rc_in_delta

        self.f_max = f_max
        self.phi_max = phi_max
        self.psidot_max = np.deg2rad(psidot_max)

        self.R_max = np.sin(np.deg2rad(phi_max))

    def set_throttle_rp_y_rate(self, rc_in):
        self.throttle_in = (self.f_max * float(rc_in[2] - self.rc_in_min)
                            / float(2*self.rc_in_delta))

        self.r[0] = (float(rc_in[1] - self.rc_in_mid)
                     /float(self.rc_in_delta)
                     *self.R_max)
        self.r[1] = (float(rc_in[0] - self.rc_in_mid)
                     / float(self.rc_in_delta)
                     *self.R_max)
        self.yaw_rate = (self.psidot_max
                         * float(rc_in[3] - self.rc_in_mid)
                         /self.rc_in_delta)

    def get_throttle_rp_y_rate(self):
        return self.throttle_in, self.r, self.yaw_rate