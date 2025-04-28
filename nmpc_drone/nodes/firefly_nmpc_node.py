#! /usr/bin/env python3.8
import os
import sys

'''
Append nmpc_pkg directory using sys module
'''
dir_path = os.path.dirname(os.path.realpath(__file__))
print(dir_path)

pkg_dir = dir_path[:dir_path.rfind('/')]
print(pkg_dir)
sys.path.append(pkg_dir)

import numpy as np
from nmpc.utils import math_tools
from nmpc.ocp import FireflyOCP
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mav_msgs.msg import Actuators
from nmpc_drone.msg import ref
from std_srvs.srv import Empty

class Firefly_nmpc_node():
    def __init__(self):
        '''
        Initialize the ocp solver and store data for state, reference and so on.
        '''
        rospy.init_node('nmpc_quad', anonymous=True)

        # NMPC weight for state (Qmat) and control input (Rmat)
        Qmat = np.diag([1, 1, 1,                # position
                        0.5, 0.5, 0.5,          # linear velocity
                        0, 0.5, 0.5, 0.5,       # quaternion
                        0.05, 0.05, 0.05        # angular velocity
                        ])
        Rmat = np.diag([0.01]*6)                # Thrust

        Parameter = {'m': 1.513,
                     'J': np.array([0.0347, 0.0459, 0.0977]),
                     'l': 0.215,
                     'C_T': 8.54858e-06,
                     'C_M': 0.016 * 8.54858e-06}

        self.C_T = Parameter['C_T']

        # Create ocp solver object
        self.ocp_solver_obj = FireflyOCP(u_min=0.0, u_max=5.0,
                                             Qmat=Qmat, Rmat=Rmat,
                                             Parameter = Parameter)

        self.state = np.zeros((13,))
        self.state[6] = 1.0

        self.ref = np.zeros((13,))
        self.ref[6] = 1.0

        self.u = np.zeros((6,))
        self.rpm_des = np.zeros((6,))
        self.u_msg = Actuators()

        self.ros_setup()

        rospy.on_shutdown(self.publish_zero_control_input)

    def ros_setup(self):
        '''
        Publisher and subscriber setup
        :return: None
        '''

        # To do list
        # Construct message filter to subscribe
        # odometry, imu (quaternion and angular velocity), and reference

        self.state_sub = rospy.Subscriber('/firefly/ground_truth/odometry',
                                          Odometry,
                                          self.state_callback,
                                          queue_size=1)

        # self.imu_sub = rospy.Subscriber('/hummingbird/ground_truth/imu',
        #                                 Imu,
        #                                 self.Imu_callback,
        #                                 queue_size=10)


        self.ref_sub = rospy.Subscriber('/nmpc_hexa/ref',
                                        ref,
                                        self.ref_callback,
                                        queue_size=1)

        # Input publisher to hummingbird
        self.input_pub = rospy.Publisher('/firefly/command/motor_speed',
                                         Actuators,
                                         queue_size=1)
        self.ros_rate = rospy.Rate(100)

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

        # # Get current linear velocity
        # self.state[3] = msg.twist.twist.linear.x
        # self.state[4] = msg.twist.twist.linear.y
        # self.state[5] = msg.twist.twist.linear.z

        # Get current quaternion
        self.state[6] = msg.pose.pose.orientation.w
        self.state[7] = msg.pose.pose.orientation.x
        self.state[8] = msg.pose.pose.orientation.y
        self.state[9] = msg.pose.pose.orientation.z

        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z

        q_ChildToParent = np.array([qw, qx, qy, qz])

        rotm = math_tools.quaternion2rotm(q_ChildToParent)

        vx_ChildFrame = msg.twist.twist.linear.x
        vy_ChildFrame = msg.twist.twist.linear.y
        vz_ChildFrame = msg.twist.twist.linear.z

        v_ChildFrame = np.array([vx_ChildFrame, vy_ChildFrame, vz_ChildFrame])

        v_ParentFrame = np.matmul(rotm, v_ChildFrame)

        self.state[3] = v_ParentFrame[0]
        self.state[4] = v_ParentFrame[1]
        self.state[5] = v_ParentFrame[2]

        # Get current angular velocity
        self.state[10] = msg.twist.twist.angular.x
        self.state[11] = msg.twist.twist.angular.y
        self.state[12] = msg.twist.twist.angular.z

        # print('position: ',self.state[0], ', ', self.state[1], ', ', self.state[2])

    # def Imu_callback(self, msg):
    #
    #     # self.state[6] = msg.orientation.w
    #     # self.state[7] = msg.orientation.x
    #     # self.state[8] = msg.orientation.y
    #     # self.state[9] = msg.orientation.z
    #     #
    #     # self.state[10] = msg.angular_velocity.x
    #     # self.state[11] = msg.angular_velocity.y
    #     # self.state[12] = msg.angular_velocity.z

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

        self.ref[6] = np.cos(msg.psi_des/2.0)
        self.ref[7] = 0
        self.ref[8] = 0
        self.ref[9] = np.sin(msg.psi_des/2.0)

        self.ref[10] = 0
        self.ref[11] = 0
        self.ref[12] = msg.dpsi_des

        # print('Reference position: ', self.ref[:3])

    def publish_control_input(self):
        self.u, status = self.ocp_solver_obj.ocp_solve(self.state, self.ref)

        # u[i] = C_lift * rpm[i]^2
        # rpm[i] = sqrt(u[i]/C_lift)
        if status == 0:
            for i in range(6):
                self.rpm_des[i] = np.sqrt(self.u[i]/self.C_T)
        else:
            self.publish_zero_control_input()
            print('NMPC : Infeasible')

        self.u_msg.header.stamp = rospy.Time.now()
        self.u_msg.header.frame_id = "nmpc_node"
        self.u_msg.angular_velocities = self.rpm_des

        self.input_pub.publish(self.u_msg)

    def position_error(self):
        tracking_error = self.state[:3] - self.ref[:3]
        return np.linalg.norm(tracking_error)

    def publish_zero_control_input(self):
        print('Publishing zero control input')
        self.rpm_des[:] = 0
        self.u_msg.angular_velocities = self.rpm_des
        self.input_pub.publish(self.u_msg)

    def run(self):
        # rospy.spin()
        while not rospy.is_shutdown():
            self.publish_control_input()
            self.ros_rate.sleep()

# def main():
#     rospy.init_node('nmpc_quad', anonymous=True)
#     nmpc_quad = nmpc_quad_node()
#     ros_rate = rospy.Rate(100)
#     while not rospy.is_shutdown():
#         ros_rate.sleep()
#     # nmpc_quad.publish_zero_control_input()

if __name__ == '__main__':
    # main()
    nmpc_hexa_node = Firefly_nmpc_node()
    nmpc_hexa_node.run()