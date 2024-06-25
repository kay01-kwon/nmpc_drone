import quad_nmpc_pkg
from acados_template import AcadosOcpSolver
import rospy
from mav_msgs.msg import Actuators
from std_srvs.srv import Empty

class nmpc_quad_node:
    def __init__(self):
        ocp_obj = quad_nmpc_pkg.ocp_sovler.OcpSovler()
        self.ocp = ocp_obj.ocp_sovler.get_ocp_sovler()