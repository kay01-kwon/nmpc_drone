from acados_template import AcadosModel
import tools
import casadi as cs

class QuadModel:
    def __init__(self, m, J, l, C_lift, C_moment, model_description):
        '''
        Constructor for QuadModel
        :param m: mass
        :param J: Put np.diag([Jxx, Jyy, Jzz])
        :param l: arm length
        :param C_lift: Coefficient of lift
        :param C_moment: Coefficient of moment
        :param model_description: '+' or 'x'
        '''

        # Model name and create AcadosModel object.
        self.model_name = 'Quadrotor model'
        self.model = AcadosModel()

        self.l = l

        # Get parameters
        # mass, moment of inertia, lift coefficient, moment coefficient
        self.m = m
        self.J = J
        self.C_lift = C_lift
        self.C_moment = C_moment

        # Model description ('x' or '+')
        self.model_description = model_description

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

        self.f_expl = cs.vertcat(self.p_dynamics(), self.v_dynamics(),
                                 self.q_dynamics(), self.w_dynamics())

        self.f_impl = self.xdot - self.f_expl

        self.model.f_expl_expr = self.f_expl
        self.model.f_impl_expr = self.f_impl
        self.model.x = self.x
        self.model.xdot = self.xdot
        self.model.u = self.u
        self.model.name = self.model_name
        return self.model

    def p_dynamics(self):
        return self.v

    def v_dynamics(self):

        # Four rotor thrust: C_lift*u**2
        thrust = self.C_lift * self.u**2

        # Get the collective thrust to compute the dynamics
        collective_thrust = thrust[0] + thrust[1] + thrust[2] + thrust[3]

        # Represent it as force vector
        force = cs.vertcat(0.0, 0.0, collective_thrust)

        # Divide mass to get acceleration control input
        acc_input = force/self.m

        g_vec = cs.vertcat(0.0, 0.0, -9.81)

        # Get rotation matrix from quaternion
        rotm = tools.quaternion2rotm(self.q)

        dvdt = cs.mtimes(rotm, acc_input) - g_vec

        return dvdt


    def q_dynamics(self):
        w_quat_form = cs.vertcat(0.0, self.w)
        dqdt = tools.otimes(w_quat_form, self.q)
        return dqdt

    def w_dynamics(self):

        # Four rotor thrust: C_lift*u**2
        thrust = self.C_lift * self.u**2


        # Convert Four rotor thrusts to moment
        m_x, m_y, m_z = tools.thrust2moment(self.model_description, thrust, self.l, self.C_moment)
        M_vec = cs.vertcat(m_x , m_y, m_z)

        # J*w
        J_w = cs.vertcat(self.J[0]*self.w[0],
                         self.J[1]*self.w[1],
                         self.J[2]*self.w[2])

        # J*dwdt = M - w x (J*w)
        J_dwdt = M_vec - cs.mtimes(tools.vec2skew_symm(self.w), J_w)

        # dwdt = J^{-1}*(M - w x (Jw))
        dwdt = cs.vertcat(1/self.J[0]*J_dwdt[0],
                          1/self.J[1]*J_dwdt[1],
                          1/self.J[2]*J_dwdt[2])

        return dwdt






