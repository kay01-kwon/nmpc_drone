import os
import shutil
from acados_template import AcadosOcp, AcadosOcpSolver
from nmpc.model.firefly_model import FireflyModel
from scipy.linalg import block_diag
import numpy as np

class FireflyOCP:
    def __init__(self, u_min = 0., u_max = 5.,
                 n_nodes = 10, t_horizon = 1.0,
                 Qmat = None, Rmat = None,
                 Parameter = None):
        '''
        Constructor for OcpSolver
        :param u_min: minimum rotor thrust (N)
        :param u_max: maximum rotor thrust (N)
        :param n_nodes: Number of nodes for NMPC
        :param T_horizon: Prediction horizon
        '''

        if Qmat is None:
            Qmat = np.diag([1, 1, 1,
                            0.5, 0.5, 0.5,
                            0, 0.5, 0.5, 0.5,
                            0.05, 0.05, 0.05])
        if Rmat is None:
            Rmat = np.diag([0.01]*6)

        if Parameter is None:
            Parameter = {'m': 1.513,
                        'J': np.array([0.0347, 0.0459, 0.0977]),
                        'l': 0.215,
                        'C_T': 8.54858e-06,
                        'C_M':  0.016*8.54858e-06}

        # Set initial state
        self.x0 = np.array([
            0.0, 0.0, 0.0,          # position
            0.0, 0.0, 0.0,          # linear velocity
            1.0, 0.0, 0.0, 0.0,     # quaternion
            0.0, 0.0, 0.0           # angular velocity
        ])

        # Create AcadosOcp
        self.ocp = AcadosOcp()

        # Object generation
        hexa_model_obj = FireflyModel(Parameter)

        # Get Quad model from the quad_model_obj
        self.model = hexa_model_obj.get_acados_model()

        # Set ocp model
        self.ocp.model = self.model

        # Set min max value of the rotor speed
        self.u_min = u_min
        self.u_max = u_max

        # Set the number of nodes
        self.ocp.dims.N = n_nodes

        # Set time horizon
        self.t_horizon = t_horizon

        # Get dim of state, control input, y, and y_e
        self.nx = self.model.x.rows()
        self.nu = self.model.u.rows()
        self.ny = self.nx + self.nu
        self.ny_e = self.nx

        # OCP setup

        # 1. Set ocp cost

        # 1.1 Cost function type
        self.ocp.cost.cost_type = 'LINEAR_LS'
        self.ocp.cost.cost_type_e = 'LINEAR_LS'

        # 1.2 cost setup for state
        self.ocp.cost.Vx = np.zeros((self.ny, self.nx))
        self.ocp.cost.Vx[:self.nx, :self.nx] = np.eye(self.nx)
        self.ocp.cost.Vx_e = np.eye(self.nx)

        # 1.3 cost setup for control input
        self.ocp.cost.Vu = np.zeros((self.ny, self.nu))
        self.ocp.cost.Vu[-self.nu:,-self.nu:] = np.eye(self.nu)

        # 1.3 weight setup
        self.ocp.cost.W = block_diag(Qmat, Rmat)
        self.ocp.cost.W_e = Qmat

        # Reference setup
        self.ocp.cost.yref = np.concatenate((self.x0, np.zeros(6)))
        self.ocp.cost.yref_e = self.x0

        # self.ocp.parameter_values = np.zeros((6,))

        # 2. Set ocp constraints
        self.ocp.constraints.x0 = self.x0
        self.ocp.constraints.lbu = np.array([self.u_min]*6)
        self.ocp.constraints.ubu = np.array([self.u_max]*6)
        self.ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4, 5])

        # 3. Set ocp solver
        self.ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        self.ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        self.ocp.solver_options.integrator_type = 'ERK'
        self.ocp.solver_options.print_level = 0     # Do not print
        self.ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        self.ocp.solver_options.tf = self.t_horizon

        self.acados_ocp_solver = AcadosOcpSolver(self.ocp)

    def ocp_solve(self, state, ref, u_prev = None):

        if u_prev is None:
            u_prev = np.zeros((6,))

        y_ref = np.concatenate((
            ref,
            np.zeros((self.nu,))
        ))
        y_ref_N = ref

        # Set constraint
        self.acados_ocp_solver.set(0,
                                   "lbx",
                                   state)

        self.acados_ocp_solver.set(0,
                                   "ubx",
                                   state)

        # self.acados_ocp_solver.set(0, 'p', f)

        for stage in range(self.ocp.dims.N):
            self.acados_ocp_solver.set(stage,
                                       'y_ref',
                                       y_ref)
            # self.acados_ocp_solver.set(stage,'p', f)
        self.acados_ocp_solver.set(self.ocp.dims.N, 'y_ref', y_ref_N)

        status = self.acados_ocp_solver.solve()

        u = self.acados_ocp_solver.get(0, 'u')

        return u, status

