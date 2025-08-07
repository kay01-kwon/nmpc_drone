#! /usr/bin/env python3.8
import os
import sys

'''
Append manual control directory using sys module
'''
dir_path = os.path.dirname(os.path.realpath(__file__))
print(dir_path)

pkg_dir = dir_path[:dir_path.rfind('/')]
print(pkg_dir)
sys.path.append(pkg_dir)

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCIn
from ros_libcanard.msg import hexa_cmd_raw
from manual_control.rc_converter.rc_converter import RcConverter
from manual_control.rp_yrate_controller.rp_yrate_controller import RpYrateController
from manual_control.rp_yrate_controller.inverse_dynamics import InverseDynamics
class manual_control_node():
    def __init__(self):

        rospy.init_node('manual_control', anonymous=True)

        self.rc_state = np.zeros((12,))
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.w = np.zeros((3,))

        self.rc_subsriber = []
        self.odom_subsriber = []
        self.cmd_pub = []

        self.RcConverter = RcConverter(f_max = 0.9*9.81*6, phi_max=10, psidot_max=10)

        Kp = np.diag([4, 4, 0.3])
        Kd = np.diag([1, 1, 0.01])
        J = np.diag([0.052, 0.052, 0.070])
        self.RpYrateController = RpYrateController(Kp, Kd, J)

        param = {'arm_length': 0.265,
                 'rotor_const': 1.456e-07,
                'moment_const': 0.01569,
                'T_min': 0.100*9.81,
                'T_max': 0.90*9.81}

        self.InverseDynamics = InverseDynamics(param)

        self.MaxBit = 8191
        self.MaxRPM = 9800

        self.u_msg = hexa_cmd_raw()

        self.ros_setup()

    def ros_setup(self):
        self.rc_subscriber = rospy.Subscriber('/mavros/rc/in',
                                              RCIn,
                                              self.rc_callback)
        self.odom_subscriber = rospy.Subscriber('/custom_hexacopter/ground_truth/odometry'
                                                , Odometry
                                                , self.odom_callback)
        self.cmd_pub = rospy.Publisher('/uav/cmd_raw',
                                       hexa_cmd_raw,
                                       queue_size=10)

    def rc_callback(self, rc_msg):
        self.rc_state = rc_msg.channels
        self.RcConverter.set_throttle_rp_y_rate(self.rc_state)
        f, r, y_rate = self.RcConverter.get_throttle_rp_y_rate()

        self.RpYrateController.set_rp_yrate(r, y_rate, self.q, self.w)
        M = self.RpYrateController.compute_moment()

        rotor_speed = self.InverseDynamics.compute_des_rpm(f,M)
        print(rotor_speed)
        for i in range(6):
            self.u_msg.raw[i] = int(rotor_speed[i]*self.MaxBit/self.MaxRPM)
        self.cmd_pub.publish(self.u_msg)


    def odom_callback(self, odom_msg):
        self.q[0] = odom_msg.pose.pose.orientation.w
        self.q[1] = odom_msg.pose.pose.orientation.x
        self.q[2] = odom_msg.pose.pose.orientation.y
        self.q[3] = odom_msg.pose.pose.orientation.z

        self.w[0] = odom_msg.twist.twist.angular.x
        self.w[1] = odom_msg.twist.twist.angular.y
        self.w[2] = odom_msg.twist.twist.angular.z


if __name__ == '__main__':
    print('manual control')
    manual_control_obj = manual_control_node()
    rospy.spin()