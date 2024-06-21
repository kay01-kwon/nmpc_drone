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

        # Casadi: Set position, linear velocity
        # quaternion and angular velocity
        self.p = cs.MX.sym('p',3)
        self.v = cs.MX.sym('v',3)
        self.q = cs.MX.sym('q',4)
        self.w = cs.MX.sym('w',3)
        

