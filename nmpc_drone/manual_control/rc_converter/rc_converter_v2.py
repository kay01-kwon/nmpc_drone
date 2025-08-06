import numpy as np

class RcConverterV2:
    def __init__(self, ax_max, ay_max, vz_max, psidot_max):
        # Maximum acceleration, altitude rate and yaw rate (deg/s)
        # Initialize rc input as zeros

        # Maximum acceleration and velocity
        self.ax_max_ = ax_max
        self.ay_max_ = ay_max
        self.R_max_ = np.sqrt(ax_max**2 + ay_max**2)
        self.vz_max_ = vz_max

        # Maximum yaw rate
        self.psidot_max_ = np.deg2rad(psidot_max)

        self.a_ref_ = np.zeros((3,))
        self.v_ref_ = np.zeros((3,))

        self.wd_ref_ = np.zeros((3,))

        self.rc_in_mid_ = 1500
        self.rc_in_delta_ = 512
        self.rc_in_min_ = self.rc_in_mid_ - self.rc_in_delta_
        self.rc_in_max_ = self.rc_in_mid_ + self.rc_in_delta_

        self.a_in_ = np.zeros((3,))

    def set_throttle_rp_y_rate(self, rc_in):

        self.a_in_[0] = self.ax_max_*(float(rc_in[1] - self.rc_in_mid_)
                     /float(self.rc_in_delta_))

        self.a_in_[1] = self.ay_max_*(float(rc_in[0] - self.rc_in_mid_)
                        / float(self.rc_in_delta_))

        R = np.sqrt(self.a_in_[0]**2 + self.a_in_[1]**2)

        if R > self.R_max_:
            self.a_ref_ = (self.R_max_*self.a_in_
                          /np.sqrt(self.a_in_[0]**2 + self.a_in_[1]**2))
        else:
            self.a_ref_ = self.a_in_

        self.v_ref_[2] = self.vz_max_*(self.vz_max_ *
                                       float(rc_in[2] - self.rc_in_min_)
                            / float(2*self.rc_in_delta_))

        self.wd_ref_[2] = (self.psidot_max_
                         * float(rc_in[3] - self.rc_in_mid_)
                         /self.rc_in_delta_)

    def get_avw_rate(self):
        return self.a_ref_, self.v_ref_, self.wd_ref_[2]