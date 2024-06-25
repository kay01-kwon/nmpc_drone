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

        self.ref = self.state

        self.u = np.zeros((4,))
        self.u_msg = Actuators()


    def ros_setup(self):

        self.state_sub = rospy.Subscriber('/hummingbird/ground_truth/odometry',
                                          Odometry,
                                          self.state_callback,
                                          queue_size=1)

        self.ref_sub = rospy.Subscriber('/nmpc_quad/ref',
                                        Odometry,
                                        self.ref_callback,
                                        queue_size=1)

        # Input publisher to hummingbird
        self.input_pub = rospy.Publisher('/hummingbird/command/motor_speed',
                                         Actuators,
                                         queue_size=1)

    def state_callback(self, msg):

        # Get current position
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = msg.pose.pose.position.z

        # Get current quaternion
        self.state[3] = msg.pose.pose.orientation.quaternion.x
        self.state[4] = msg.pose.pose.orientation.quaternion.y
        self.state[5] = msg.pose.pose.orientation.quaternion.z
        self.state[6] = msg.pose.pose.orientation.quaternion.w

        # Get current linear velocity
        self.state[7] = msg.pose.pose.twist.twist.linear.x
        self.state[8] = msg.pose.pose.twist.twist.linear.y
        self.state[9] = msg.pose.pose.twist.twist.linear.z

        # Get current angular velocity
        self.state[10] = msg.pose.pose.twist.twist.angular.x
        self.state[11] = msg.pose.pose.twist.twist.angular.y
        self.state[12] = msg.pose.pose.twist.twist.angular.z

        try:
            self.u = self.ocp_solver_obj.set_state(self, self.state, self.ref)

            self.u_msg.header.stamp = rospy.Time.now()
            self.u_msg.header.frame_id = "nmpc_node"
            self.u_msg.angular_velocities = self.u

        except rospy.ROSInterruptException:
            rospy.logerr("NMPC is infeasible")

        self.input_pub.publish(self.u_msg)



    def ref_callback(self, msg):

        # Get reference position
        self.ref[0] = msg.pose.pose.position.x
        self.ref[1] = msg.pose.pose.position.y
        self.ref[2] = msg.pose.pose.position.z

        # Get ref quaternion
        self.ref[3] = msg.pose.pose.orientation.quaternion.x
        self.ref[4] = msg.pose.pose.orientation.quaternion.y
        self.ref[5] = msg.pose.pose.orientation.quaternion.z
        self.ref[6] = msg.pose.pose.orientation.quaternion.w

        # Get ref linear velocity
        self.ref[7] = msg.pose.pose.twist.twist.linear.x
        self.ref[8] = msg.pose.pose.twist.twist.linear.y
        self.ref[9] = msg.pose.pose.twist.twist.linear.z

        # Get ref angular velocity
        self.ref[10] = msg.pose.pose.twist.twist.angular.x
        self.ref[11] = msg.pose.pose.twist.twist.angular.y
        self.ref[12] = msg.pose.pose.twist.twist.angular.z

def main():
    rospy.init_node('nmpc_quad', anonymous=True)
    nmpc_quad = nmpc_quad_node()
    ros_rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        ros_rate.sleep()

if __name__ == '__main__':
    main()






