import numpy as np
from math_tools import quaternion_math
class PosControl():
    def __init__(self, DynParam, GainParam):

        # Control gain parameter setup
        # 1. Translational control gain
        self.Kp_trans = GainParam['Kp_trans']
        self.Ki_trans = GainParam['Ki_trans']
        self.Kd_trans = GainParam['Kd_trans']

        # 2. Rotational control gain
        self.Kp_rot = GainParam['Kp_rot']
        self.Kd_rot = GainParam['Kd_rot']
        self.Ki_rot = GainParam['Ki_rot']

        self.m = DynParam['m']
        self.J = DynParam['J']

        self.g_vec = np.array([0,0,-9.81])

        self.u = np.zeros((4,))
        self.i_err = np.zeros((3,))
        self.i_rot_err = np.zeros((3,))

    def control_input_compute(self, ref, state, dt):
        # Extract reference values
        p_ref = ref[0:3]
        v_ref = ref[3:6]
        psi_ref = ref[6]
        w_ref = ref[7]

        # Extract state values
        p = state[0:3]
        v = state[3:6]
        q = state[6:10]
        w = state[10:13]

        p_error = p - p_ref
        v_error = v - v_ref
        self.i_err = self.i_err + p_error * dt

        f_des = (- self.Kp_trans*p_error
                 - self.Kd_trans*v_error
                 - self.Ki_trans*self.i_err
                 - self.m*self.g_vec)

        self.u[0] = np.sqrt(f_des.transpose()@f_des)
        r = f_des/self.u[0]
        rx = r[0]
        ry = r[1]
        rz = r[2]
        sin_phi = np.sqrt(1-rz**2)

        if np.abs(sin_phi) > 1e-30:
            a = 1/sin_phi*np.array([-ry, rx, 0])
        else:
            a = np.zeros((3,))

        cos_phi_2 = np.sqrt((1+rz)/2)
        sin_phi_2 = np.sqrt((1-rz)/2)

        q_a = np.array([cos_phi_2,
                        a[0]*sin_phi_2,
                        a[1]*sin_phi_2,
                        a[2]*sin_phi_2])

        q_psi = np.array([np.cos(psi_ref/2),
                          0,
                          0,
                          np.sin(psi_ref/2)])

        q_ref = quaternion_math.otimes(q_a, q_psi)
        q_ref_conj = quaternion_math.conjugate(q_ref)
        q_err = quaternion_math.otimes(q_ref_conj, q)

        q_err_vec = self.signum(q_err[0])*q_err[1:4]
        w_error = w - w_ref

        self.i_rot_err = self.i_rot_err + q_err_vec*dt
        M = (-self.Kp_rot*q_err_vec
             - self.Kd_rot*w_error
             - self.Ki_rot*self.i_rot_err
             + np.cross(w,self.J*w))
        self.u[1:] = M

        return self.u

    def signum(self, value):
        return 1 if value > 0 else -1