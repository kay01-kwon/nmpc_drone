import numpy as np

class InverseDynamics:
    def __init__(self, Param):
        l = Param['arm_length']
        self.C_T = Param['rotor_const']
        k_m = Param['moment_const']
        self.T_max = Param['T_max']
        self.T_min = Param['T_min']

        cos_pi_3 = np.cos(np.pi/3)
        sin_pi_3 = np.sin(np.pi/3)

        ly1 = l*cos_pi_3
        ly2 = l
        ly3 = l*cos_pi_3

        ly4 = -l*cos_pi_3
        ly5 = -l
        ly6 = -l*cos_pi_3

        lx1 = l*sin_pi_3
        lx2 = 0
        lx3 = -l*sin_pi_3

        lx4 = -l*sin_pi_3
        lx5 = 0
        lx6 = l*sin_pi_3

        K_forward = np.array([
            [1, 1, 1, 1, 1, 1],
            [ly1, ly2, ly3, ly4, ly5, ly6],
            [-lx1, -lx2, -lx3, -lx4, -lx5, -lx6],
            [-k_m, k_m, -k_m, k_m, -k_m, k_m]
        ])

        self.K_inv = np.linalg.pinv(K_forward)

    def compute_des_rpm(self, f, M):
        fm = np.hstack([f,M])
        # print(fm)
        rotors_thrust = self.K_inv @ fm
        rotors_thrust = self.clamp(rotors_thrust)
        rotors_speed = np.sqrt(rotors_thrust/self.C_T)
        return rotors_speed

    def clamp(self, rotors_thrust):
        for i in range(len(rotors_thrust)):
            if self.T_max < rotors_thrust[i]:
                rotors_thrust[i] = self.T_max
            if self.T_min > rotors_thrust[i]:
                rotors_thrust[i] = self.T_min
        return rotors_thrust