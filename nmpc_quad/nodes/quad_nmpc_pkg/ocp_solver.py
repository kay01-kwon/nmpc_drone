from acados_template import AcadosOcp, AcadosOcpSolver
import quad_model
import scipy.linalg
import numpy as np
import casadi as cs

# Initial state
X0 = np.array([
    0.0, 0.0, 0.0,          # position
    0.0, 0.0, 0.0,          # velocity
    0.0, 0.0, 0.0, 1.0,     # quaternion
    0.0, 0.0, 0.0           # angular velocity
])


class OcpSolver():
    def __init__(self, N = 20, T_horizon = 2.0):
        # Create AcadosOcp
        self.ocp = AcadosOcp()


        # Object generation
        quad_model_obj = quad_model.QuadModel(m = 1,
                                          J =np.diag([1.0, 1.0, 1.0]),
                                          l = 0.2,
                                          C_lift = 1,
                                          C_moment = 1,
                                          model_description = '+')

        # Get Quad model from the quad_model_obj
        self.model = quad_model_obj.get_acados_model()

        # Set ocp model
        self.ocp.model = self.model

        # Set horizon
        self.N = N
        self.ocp.dims.N = self.N

        # Get the dimension of state, input, y and y_e (terminal)
        self.nx = self.model.x.rows()
        self.nu = self.model.u.rows()
        self.ny = self.nx + self.nu
        self.ny_e = self.nx

        # cost Q:
        # px, py, pz,
        # vx, vy, vz,
        # qx, qy, qz, qw
        # wx, wy, wz
        self.Q_mat = 2*np.diag([1.0, 1.0, 1.0,
                                0.05, 0.05, 0.05,
                                0.1, 0.1, 0.1,
                                0.05, 0.05, 0.05])




