from acados_template import AcadosModel
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

        # Casadi: Assign u (control input)
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

    def v_dynamics(self):
        




