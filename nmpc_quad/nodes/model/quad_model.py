from acados_template import AcadosModel
import numpy as np
import casadi as cs

class QuadModel:
    def __init__(self, m, J, C_lift, C_moment):

        # Model name
        self.model_name = 'Quadrotor model'

        # Get parameters
        # mass, moment of inertia, lift coefficient, moment coefficient
        self.m = m
        self.J = J
        self.C_lift = C_lift
        self.C_moment = C_moment

        # Casadi: Assign state
        self.p = cs.MX.sym('p',3)   # position
        self.v = cs.MX.sym('v',3)   # velocity
        self.q = cs.MX.sym('q',4)   # quaternion
        self.w = cs.MX.sym('w',3)   # angular velocity
        self.x = cs.vertcat(self.p, self.v, self.q, self.w)     #state
        self.x_dim = 13

        # Casadi: Assign four rotors' thrust control input
        self.u1 = cs.MX.sym('u1')
        self.u2 = cs.MX.sym('u2')
        self.u3 = cs.MX.sym('u3')
        self.u4 = cs.MX.sym('u4')
        self.u = cs.vertcat(self.u1, self.u2, self.u3, self.u4)
        self.u_dim = 4
        




