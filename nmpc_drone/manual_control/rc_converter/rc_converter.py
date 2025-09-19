import numpy as np
from enum import Enum

class FlightMode(Enum):
    ARMED = 1
    AUTO = 2
    MANUAL_STAB = 3
    KILL = 4

class RcConverter:
    def __init__(self, manualParam):
        # Set flight mode
        self.mode_ = FlightMode.AUTO

        # Set maximum acceleration and altitude
        self.a_max_ = manualParam['a_max']
        self.z_max_ = manualParam['z_max']

        self.R_max_ = np.sqrt(2*self.a_max_**2)

        self.dpsi_dt_max_ = manualParam['dpsi_dt_max']

        self.rc_in_mid_ = 1500
        self.rc_in_delta_ = 512
        self.rc_in_min_ = self.rc_in_mid_ - self.rc_in_delta_
        self.rc_in_max_ = self.rc_in_mid_ + self.rc_in_delta_

        # a_des_ = [ax_des, ay_des]
        self.a_des_ = np.zeros((2,))
        self.z_des_ = np.zeros((1,))
        self.dpsi_dt_des_ = np.zeros((1,))

    def set_rc(self, rc_in):
        ax_temp =  self.a_max_ * self._constrain(rc_in[1])
        ay_temp = -self.a_max_ * self._constrain(rc_in[0])

        R_temp = np.sqrt(ax_temp**2 + ay_temp**2)

        scale = self.R_max_ / R_temp

        if R_temp > self.R_max_ and R_temp > 1e-9:
            ax_des = scale * ax_temp
            ay_des = scale * ay_temp
        else:
            ax_des = ax_temp
            ay_des = ay_temp

        self.a_des_[0] = ax_des
        self.a_des_[1] = ay_des
        self.z_des_ = (self.z_max_
                       * self._altitude_constrain(rc_in[2]))
        self.dpsi_dt_des_ = (- self.dpsi_dt_max_
                             * self._constrain(rc_in[3]))


        if self._two_pos(rc_in[9]) == 'LOW':
            self.mode_ = FlightMode.KILL
        else:
            if self._three_pos(rc_in[5]) == 'LOW':
                self.mode_ = FlightMode.MANUAL_STAB
            elif self._three_pos(rc_in[5]) == 'HIGH':
                self.mode_ = FlightMode.AUTO
            else:
                self.mode_ = FlightMode.ARMED

    def get_rc_state(self):
        return self.a_des_, self.z_des_, self.dpsi_dt_des_, self.mode_

    def _constrain(self, input):
        temp = (float(input-self.rc_in_mid_)/
        float(self.rc_in_delta_))
        return temp

    def _altitude_constrain(self, input):
        temp = (float(input - self.rc_in_min_)/
                float(2*self.rc_in_delta_))
        return temp

    def _two_pos(self, pwm:int) -> str:
        '''
        Return 'LOW' | 'HIGH'
        :param pwd:
        :return:
        '''
        if pwm < 1200:
            return 'LOW'
        elif pwm > 1700:
            return 'HIGH'

    def _three_pos(self, pwm:int) -> str:
        '''
        Return 'LOW' | 'MID' |'HIGH'
        :param pwm:
        :return:
        '''
        if pwm < 1200:
            return 'LOW'
        elif pwm > 1700:
            return 'HIGH'
        else:
            return 'MID'