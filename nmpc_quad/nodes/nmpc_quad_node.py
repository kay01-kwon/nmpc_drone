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
        self.state[6] = 1.0

        self.y_ref = np.zeros((17,))
        self.y_ref_N = np.zeros((17,))

        self.u_msg = Actuators()

    def ros_setup(self):

        self.state_sub = rospy.Subscriber('/hummingbird/ground_truth/odometry',
                                          Odometry, self.state_callback,
                                          queue_size=1)

        # Input publisher to hummingbird
        self.input_pub = rospy.Publisher('/hummingbird/command/motor_speed',
                                         Actuators,
                                         queue_size=1)

    def state_callback(self, msg):

        # Get state
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = msg.pose.pose.position.z

        self.state[3] = msg.pose.pose.orientation.quaternion.x
        self.state[4] = msg.pose.pose.orientation.quaternion.y
        self.state[5] = msg.pose.pose.orientation.quaternion.z
        self.state[6] = msg.pose.pose.orientation.quaternion.w

        self.state[7] = msg.pose.pose.twist.twist.linear.x
        self.state[8] = msg.pose.pose.twist.twist.linear.y
        self.state[9] = msg.pose.pose.twist.twist.linear.z

        self.state[10] = msg.pose.pose.twist.twist.angular.x
        self.state[11] = msg.pose.pose.twist.twist.angular.y
        self.state[12] = msg.pose.pose.twist.twist.angular.z







