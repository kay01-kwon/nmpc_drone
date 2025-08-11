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

from manual_control.rc_converter.rc_converter_alt import RcConverterAlt
from manual_control.rp_yrate_controller.rp_alt_controller import RpyzController
from math_tools.inverse_dynamics import InverseDynamics
from math_tools import quaternion_math
from geometry_msgs.msg import Wrench

def low_pass_filter(data_prev, data_current, alpha):
    filtered = alpha*data_prev + (1-alpha) * data_current
    return filtered
class manual_control_node_alt():
    def __init__(self):

        rospy.init_node('manual_control_alt', anonymous=True)

        self.rc_state = np.zeros((12,))
        self.p = np.zeros((3,))
        self.v = np.zeros((3,))
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.w = np.zeros((3,))
        self.tau = np.zeros((3,))
        self.r_off = np.zeros((3,))

        self.rc_subsriber = []
        self.odom_subsriber = []
        self.cmd_pub = []

        self.MaxBit = 8191
        self.MaxRPM = 9800

        self.rotor_speed_filtered = np.zeros((6,))

        self.RcConverter = RcConverterAlt(ax_max = 0.3, ay_max = 0.3,
                                         z_max = 0.9, psidot_max = 10)

        GainParam = {'Kp_trans': np.diag([0,0,4]),
                     'Kd_trans': np.diag([0,0,1]),
                     'Kp_rot': np.diag([3, 3, 0.05]),
                     'Kd_rot': np.diag([0.52, 0.52, 0.025])}
        DynParam = {'m': 2.9,
                    'J': np.diag([0.052, 0.052, 0.080])}

        self.RpyzController = RpyzController(GainParam, DynParam)

        param = {'arm_length': 0.265,
                 'rotor_const': 1.465e-07,
                'moment_const': 0.01569,
                'T_min': 0.10*9.81,
                'T_max': 0.90*9.81}

        self.InverseDynamics = InverseDynamics(param)

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
                                       queue_size=1)
        self.wrench_subscriber = rospy.Subscriber('/ekf_wrench',
                                                  Wrench,
                                                  self.wrench_callback)

    def rc_callback(self, rc_msg):
        self.rc_state = rc_msg.channels
        self.RcConverter.set_throttle_rp_y_rate(self.rc_state)
        a_ref, z_ref, w_rate = self.RcConverter.get_avw_rate()

        self.RpyzController.set_rp_yrate(a_ref, z_ref, w_rate,
                                         self.p, self.v, self.q, self.w)
        # self.tau = np.zeros((3,))
        u = self.RpyzController.compute_moment(self.tau)

        rotor_speed = self.InverseDynamics.compute_des_rpm(u[0],u[1:])
        print(rotor_speed)
        for i in range(6):
            self.u_msg.raw[i] = int(rotor_speed[i]*self.MaxBit/self.MaxRPM)
        if self.rc_state[8] == 2000:
            for i in range(6):
                self.u_msg.raw[i] = 0
        self.cmd_pub.publish(self.u_msg)


    def odom_callback(self, odom_msg):

        self.p[0] = odom_msg.pose.pose.position.x
        self.p[1] = odom_msg.pose.pose.position.y
        self.p[2] = odom_msg.pose.pose.position.z - 0.3

        v_temp = np.zeros((3,))
        v_temp[0] = odom_msg.twist.twist.linear.x
        v_temp[1] = odom_msg.twist.twist.linear.y
        v_temp[2] = odom_msg.twist.twist.linear.z

        self.q[0] = odom_msg.pose.pose.orientation.w
        self.q[1] = odom_msg.pose.pose.orientation.x
        self.q[2] = odom_msg.pose.pose.orientation.y
        self.q[3] = odom_msg.pose.pose.orientation.z

        rotm = quaternion_math.quaternion_to_rotm(self.q)

        self.v = rotm@v_temp

        # print(self.v)

        self.w[0] = odom_msg.twist.twist.angular.x
        self.w[1] = odom_msg.twist.twist.angular.y
        self.w[2] = odom_msg.twist.twist.angular.z

    def wrench_callback(self, wrench_msg):
        f = wrench_msg.force.z
        # self.tau[0] = wrench_msg.torque.x
        # self.tau[1] = wrench_msg.torque.y
        # self.tau[2] = wrench_msg.torque.z
        tau_signal = np.array([wrench_msg.torque.x,
                               wrench_msg.torque.y,
                               wrench_msg.torque.z])

        self.tau = low_pass_filter(self.tau, tau_signal, 0.0)

        # print(self.r_off)

if __name__ == '__main__':
    print('manual control')
    manual_control_obj = manual_control_node_alt()
    rospy.spin()