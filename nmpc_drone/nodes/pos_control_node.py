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
import rospy
from math_tools.inverse_dynamics import InverseDynamics
from nmpc.utils import math_tools
from pos_control.pid_control import PosControl
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from ros_libcanard.msg import hexa_cmd_raw
from nmpc_drone.msg import ref


def low_pass_filter(data_prev, data_current, alpha):
    filtered = alpha*data_prev + (1-alpha) * data_current
    return filtered

class PosControlNode():

    def __init__(self):

        rospy.init_node('pos_control_node', anonymous=True)

        DynParam, GainParam, InverseDynParam = self.setup_pid_control_param()

        print('DynParam', DynParam)
        print('GainParam', GainParam)

        self.alpha = GainParam['alpha']
        self.PidControl = PosControl(DynParam=DynParam,
                                     GainParam=GainParam)

        self.InverseDyn = InverseDynamics(InverseDynParam)

        self.state = np.zeros((13,))
        self.state[6] = 1.0
        self.ref = np.zeros((10,))
        self.tau = np.zeros((3,))

        self.u = np.zeros((6,))
        self.u_msg = hexa_cmd_raw()
        self.MaxBit = 8191
        self.MaxRPM = 9800

        self.t_now = rospy.get_time()
        self.t_prev = self.t_now

        self.is_first_flight = False

        self.ros_setup()

    def ros_setup(self):

        self.state_sub = rospy.Subscriber('/custom_hexacopter/ground_truth/odometry',
                                          Odometry,
                                          self.state_callback,
                                          queue_size=1)

        self.ref_sub = rospy.Subscriber('/pid_hexa/ref',
                                        ref,
                                        self.pid_ref_callback,
                                        queue_size=1)
        self.wrench_sub = rospy.Subscriber('/wrench',
                                           WrenchStamped,
                                           self.wrench_callback)

        self.input_pub = rospy.Publisher('/uav/cmd_raw',
                                         hexa_cmd_raw,
                                         queue_size=1)

        self.ros_rate = rospy.Rate(100)

    def state_callback(self, msg):
        '''
        State call back function
        :param msg: Odometry message
        Solve control input for quadrotor
        '''

        # Get current position
        self.time_stamp_current = msg.header.stamp.to_sec()
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = msg.pose.pose.position.z - 0.325

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

        v_ParentFrame = rotm@v_ChildFrame

        self.state[3] = v_ParentFrame[0]
        self.state[4] = v_ParentFrame[1]
        self.state[5] = v_ParentFrame[2]

        # Get current angular velocity
        self.state[10] = msg.twist.twist.angular.x
        self.state[11] = msg.twist.twist.angular.y
        self.state[12] = msg.twist.twist.angular.z

        self.publish_control_input()

    def pid_ref_callback(self, msg):
        for i in range(3):
            self.ref[i] = msg.p_des[i]
            self.ref[i+3] = msg.v_des[i]
        self.ref[6] = msg.psi_des
        self.ref[7] = msg.dpsi_des

    def wrench_callback(self, msg):
        f = msg.wrench.force.z
        tau_new = np.array([msg.wrench.torque.x,
                            msg.wrench.torque.y,
                            msg.wrench.torque.z])

        self.tau = low_pass_filter(self.tau, tau_new, self.alpha)
    def publish_control_input(self):
        self.t_now = rospy.get_time()
        dt = self.t_now - self.t_prev

        self.u = self.PidControl.control_input_compute(ref=self.ref,
                                                  state=self.state,
                                                  dt=dt)


        self.u[1:] = self.u[1:] - self.tau

        rotor_speed = self.InverseDyn.compute_des_rpm(self.u[0],
                                                      self.u[1:])

        for i in range(6):
            self.u_msg.raw[i] = int(rotor_speed[i]*self.MaxBit/self.MaxRPM)

        # if self.ref[2] == 0:
        #     for i in range(6):
        #         self.u_msg.raw[i] = int(2000)

        self.input_pub.publish(self.u_msg)
        self.t_prev = self.t_now

    def setup_pid_control_param(self):
        node_name = rospy.get_name()

        # Get nominal dynamics parameter
        m = rospy.get_param(node_name + '/nominal_dynamics/m')
        J = rospy.get_param(node_name + '/nominal_dynamics/J')
        r_off = rospy.get_param(node_name + '/nominal_dynamics/r_off')
        l = rospy.get_param(node_name + '/nominal_dynamics/l')

        # Get aerodynamics parameter
        C_T = rospy.get_param(node_name + '/aerodynamics/C_T')
        k_m = rospy.get_param(node_name + '/aerodynamics/k_m')

        # Get translational control gain (PID)
        Kp_trans = rospy.get_param(node_name + '/pid/trans/Kp')
        Kd_trans = rospy.get_param(node_name + '/pid/trans/Kd')
        Ki_trans = rospy.get_param(node_name + '/pid/trans/Ki')

        # Get rotational control gain (PD)
        Kp_rot = rospy.get_param(node_name + '/pid/rot/Kp')
        Kd_rot = rospy.get_param(node_name + '/pid/rot/Kd')
        Ki_rot = rospy.get_param(node_name + '/pid/rot/Ki')

        # Get weight of low pass filter (alpha)
        alpha = rospy.get_param(node_name + '/pid/alpha')

        DynParam = {'m': m, 'J': J, 'r_off': r_off, 'l':l}

        PidGain = {'Kp_trans': np.array(Kp_trans),
                   'Kd_trans': np.array(Kd_trans),
                   'Ki_trans': np.array(Ki_trans),
                   'Kp_rot': np.array(Kp_rot),
                   'Kd_rot': np.array(Kd_rot),
                   'Ki_rot': np.array(Ki_rot),
                   'alpha': alpha}

        InverseDynParam = {'arm_length': l,
                 'rotor_const': C_T,
                'moment_const': k_m,
                'T_min': 0.05*9.81,
                'T_max': 0.90*9.81}

        return DynParam, PidGain, InverseDynParam



if __name__ == '__main__':
    print('pid control')
    node = PosControlNode()
    rospy.spin()