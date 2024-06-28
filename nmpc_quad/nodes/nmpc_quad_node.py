#! /usr/bin/env python3.8
import os
import sys
import shutil

'''
Append nmpc_pkg directory using sys module
'''
dir_path = os.path.dirname(os.path.realpath(__file__))
# print(dir_path)

pkg_dir = dir_path + '/nmpc_pkg'
# print(pkg_dir)
sys.path.append(dir_path + '/nmpc_pkg')

import numpy as np
from nmpc_pkg import ocp_solver
import rospy
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from nmpc_quad.msg import nmpc_ref

from std_srvs.srv import Empty

class nmpc_quad_node:
    def __init__(self):
        '''
        Initialize the ocp solver and store data for state, reference and so on.
        '''

        # Create ocp solver object
        self.ocp_solver_obj = ocp_solver.OcpSolver()

        self.state = np.zeros((13,))
        self.state[6] = 1.0

        self.ref = np.zeros((13,))

        self.u = np.zeros((4,))
        self.rpm_des = np.zeros((4,))
        self.u_msg = Actuators()

        self.C_lift = 8.54858e-06

        self.ros_setup()


    def ros_setup(self):
        '''
        Publisher and subscriber setup
        :return:
        '''
        self.state_sub = rospy.Subscriber('/hummingbird/ground_truth/odometry',
                                          Odometry,
                                          self.state_callback,
                                          queue_size=1)

        self.ref_sub = rospy.Subscriber('/nmpc_quad/ref',
                                        nmpc_ref,
                                        self.ref_callback,
                                        queue_size=1)

        # Input publisher to hummingbird
        self.input_pub = rospy.Publisher('/hummingbird/command/motor_speed',
                                         Actuators,
                                         queue_size=1)

    def state_callback(self, msg):
        '''
        State call back function
        :param msg: Odometry message
        Solve NMPC for quadrotor
        '''
        # Get current position
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = msg.pose.pose.position.z

        # Get current linear velocity
        self.state[3] = msg.twist.twist.linear.x
        self.state[4] = msg.twist.twist.linear.y
        self.state[5] = msg.twist.twist.linear.z

        # Get current quaternion
        self.state[6] = msg.pose.pose.orientation.w
        self.state[7] = msg.pose.pose.orientation.x
        self.state[8] = msg.pose.pose.orientation.y
        self.state[9] = msg.pose.pose.orientation.z

        # Get current angular velocity
        self.state[10] = msg.twist.twist.angular.x
        self.state[11] = msg.twist.twist.angular.y
        self.state[12] = msg.twist.twist.angular.z

        # print('position: ',self.state[0], ', ', self.state[1], ', ', self.state[2])

        status, self.u = self.ocp_solver_obj.ocp_solve(self.state, self.ref)

        # u[i] = C_lift * rpm[i]^2
        # rpm[i] = sqrt(u[i]/C_lift)
        if status != 0:
            for i in range(4):
                self.rpm_des[i] = np.sqrt(self.u[i]/self.C_lift)
        else:
            for i in range(4):
                self.rpm_des[i] = 0
            print('NMPC : Infeasible')

        self.u_msg.header.stamp = rospy.Time.now()
        self.u_msg.header.frame_id = "nmpc_node"
        self.u_msg.angular_velocities = self.rpm_des


        self.input_pub.publish(self.u_msg)



    def ref_callback(self, msg):
        '''
        Callback function for reference
        :param msg: nmpc_pkg/ref.msg
        Store reference values to the self.ref
        '''
        # Get reference position
        for i in range(3):
            self.ref[i] = msg.p_des[i]
            self.ref[i+3] = msg.v_des[i]
            self.ref[i+10] = msg.w_des[i]

        for j in range(4):
            self.ref[j+6] = msg.q_des[j]

        # print('Reference position: ', self.ref[:3])

def main():
    rospy.init_node('nmpc_quad', anonymous=True)
    nmpc_quad = nmpc_quad_node()
    ros_rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        ros_rate.sleep()

if __name__ == '__main__':
    main()






