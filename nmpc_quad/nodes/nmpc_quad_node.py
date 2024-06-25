import numpy as np

from nmpc_pkg import ocp_solver
import rospy
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from std_srvs.srv import Empty

class nmpc_quad_node:
    def __init__(self):
        # Create ocp solver object
        self.ocp_solver_obj = ocp_solver.OcpSolver()

        self.state = np.zeros((13,))

        self.y_ref = np.zeros((17,))
        self.y_ref_N = np.zeros((17,))

        self.u_msg = Actuators()

    def ros_setup(self):

        # Input publisher to hummingbird
        self.input_pub = rospy.Publisher('/hummingbird/command/motor_speed',
                                         Actuators,
                                         queue_size=1)

    def state_callback(self, msg):


