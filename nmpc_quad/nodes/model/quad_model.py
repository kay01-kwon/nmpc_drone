from acados_template import AcadosModel
import tools
import numpy as np
import casadi as cs


g = 9.81
class QuadModel:
    def __init__(self, m, J, C_lift, C_moment):

        # Model name and create AcadosModel object.
        self.model_name = 'Quadrotor model'
        self.model = AcadosModel()

        # Get parameters
        # mass, moment of inertia, lift coefficient, moment coefficient
        self.m = m
        self.J = J
        self.C_lift = C_lift
        self.C_moment = C_moment

        # Casadi: Assign x (state)
        self.p = cs.MX.sym('p',3)   # position
        self.v = cs.MX.sym('v',3)   # velocity
        self.q = cs.MX.sym('q',4)   # quaternion
        self.w = cs.MX.sym('w',3)   # angular velocity
        self.x = cs.vertcat(self.p, self.v, self.q, self.w)     #state
        self.x_dim = 13

        # Casadi: Assign u (control input: rotor speed)
        self.u1 = cs.MX.sym('u1')
        self.u2 = cs.MX.sym('u2')
        self.u3 = cs.MX.sym('u3')
        self.u4 = cs.MX.sym('u4')
        self.u = cs.vertcat(self.u1, self.u2, self.u3, self.u4)
        self.u_dim = 4

        # Casadi: Assign xdot (The differentiation of the state)
        self.dpdt = cs.MX.sym('dpdt',3)
        self.dvdt = cs.MX.sym('dvdt',3)
        self.dqdt = cs.MX.sym('dqdt',4)
        self.dwdt = cs.MX.sym('dwdt',3)
        self.xdot = cs.vertcat(self.dpdt, self.dvdt, self.dwdt, self.dqdt)


    def get_acados_model(self):


        return self.model

    def p_dynamics(self):
        return self.v

    def v_dynamics(self, sigma):

        # Four rotor thrust: C_lift*u**2
        thrust = self.C_lift * self.u**2

        # Get the collective thrust to compute the dynamics
        collective_thrust = thrust[0] + thrust[1] + thrust[2] + thrust[3]

        # Represent it as force vector
        force = cs.vertcat(0.0, 0.0, collective_thrust)

        # Divide mass to get acceleration control input
        acc_input = force/self.m

        g_vec = cs.vertcat(0.0, 0.0, g)

        # Get rotation matrix from quaternion
        rotm = tools.quat2rotmat(self.q)

        dvdt = rotm*acc_input - g + sigma/self.m
        return dvdt


    def q_dynamics(self):
        w_quat_form = cs.vertcat(0.0, self.w)
        dqdt = tools.orientmat(w_quat_form, self.q)
        return dqdt

    def w_dynamics(self, theta):

        # Four rotor thrust: C_lift*u**2
        thrust = self.C_lift * self.u**2

        Moment = 

        thrust = self.C_lift






