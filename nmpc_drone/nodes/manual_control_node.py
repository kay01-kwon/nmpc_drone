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

import threading
from utils import CustomQueue

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCIn
from ros_libcanard.msg import hexa_cmd_raw

from manual_control.rc_converter import RcConverter
from manual_control.rc_controller import RcController
from utils.inverse_dynamics import InverseDynamics

from utils import quaternion_math
from geometry_msgs.msg import WrenchStamped

def rcIn_parsing(msg:RCIn):
    return np.array([msg.header.stamp.to_sec(),
                     msg.channels])
def odom_parsing(msg:Odometry):
    return np.array([msg.header.stamp.to_sec(),
                     msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     msg.pose.pose.position.z,
                     msg.twist.twist.linear.x,
                     msg.twist.twist.linear.y,
                     msg.twist.twist.linear.z,
                     msg.pose.pose.orientation.w,
                     msg.pose.pose.orientation.x,
                     msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z])

def wrench_parsing(msg:WrenchStamped):
    return np.array([msg.header.stamp.to_sec(),
                     msg.wrench.force.x,
                     msg.wrench.force.y,
                     msg.wrench.force.z,
                     msg.wrench.torque.x,
                     msg.wrench.torque.y,
                     msg.wrench.torque.z])
class ManualControlNode():
    def __init__(self):

        rospy.init_node('manual_control_node')

        p = np.zeros((3,))
        v = np.zeros((3,))
        q = np.array([1.0,0.0,0.0,0.0])
        w = np.zeros((3,))

        self.state = np.array([p,v,q,w])

        self.tau = np.zeros((3,))

        self.MaxBit = 8191
        self.MaxRpm = 9800

        self.rc_in_buffer_ = CustomQueue(maxsize=10)
        self.mocap_state_buffer_ = CustomQueue(maxsize=10)
        self.wrench_buffer_ = CustomQueue(maxsize=10)

        manualParam = self._setup_rc_param()
        GainParam, DynParam = self._setup_gain_param()
        droneParam = self._setup_drone_param()

        # RC converter, controller and inverse dynamics object
        self.RcConverter = RcConverter(manualParam=manualParam)
        self.RcController = RcController(GainParam=GainParam,
                                         DynParam=DynParam)
        self.InverseDynamics = InverseDynamics(Param=droneParam)


        # Subscriber setup
        self.rcIn_sub = rospy.Subscriber('/mavros/rc/in',
                                         data_class=RCIn,
                                         queue_size=10,
                                         callback=self._rcCallback,
                                         tcp_nodelay=True)
        self.state_sub = rospy.Subscriber('/mavros/odometry/in',
                                          data_class=Odometry,
                                          queue_size=10,
                                          callback=self._mocapCallback,
                                          tcp_nodelay=True)
        self.wrench_sub = rospy.Subscriber('/wrench',
                                           data_class=WrenchStamped,
                                           queue_size=10,
                                           callback=self._wrenchCallback,
                                           tcp_nodelay=True)

        # Publisher
        self.cmd_pub = rospy.Publisher('/uav/cmd_raw',
                                       data_class=hexa_cmd_raw,
                                       queue_size=10)

        self.u_msg = hexa_cmd_raw()

        self.cv_ = threading.Condition()
        self.period_ = 0.010

        self.t_curr_ = rospy.get_time()
        self.t_prev_ = self.t_curr_

        # RC, mocap state, wrench in turn
        self.time_latest_ = np.array([-1e3,
                                      -1e3,
                                      -1e3])
        self.latest_tx_wall_ = np.zeros((4,))
        self.time_out_ = np.array([0.015,
                                   0.015,
                                   0.015])

    def _rcCallback(self, msg:RCIn):
        with self.cv_:
            rc_temp = rcIn_parsing(msg)
            if self.rc_in_buffer_.full():
                self.rc_in_buffer_.pop()
                self.rc_in_buffer_.push(rc_temp)
            else:
                self.rc_in_buffer_.push(rc_temp)

            self.cv_.notify_all()
    def _mocapCallback(self, msg:Odometry):
        with self.cv_:
            mocap_temp = odom_parsing(msg)
            if self.mocap_state_buffer_.full():
                self.mocap_state_buffer_.pop()
                self.mocap_state_buffer_.push(mocap_temp)
            else:
                self.mocap_state_buffer_.push(mocap_temp)
            self.cv_.notify_all()

    def _wrenchCallback(self, msg:WrenchStamped):
        with self.cv_:
            wrench_temp = wrench_parsing(msg)
            if self.wrench_buffer_.full():
                self.wrench_buffer_.pop()
                self.wrench_buffer_.push(wrench_temp)
            else:
                self.wrench_buffer_.push(wrench_temp)
            self.cv_.notify_all()
    def _process_func(self):

        MAX_CATCHUP_STEPS = 3
        rospy.loginfo("Starting thread")

        while not rospy.is_shutdown():

            with self.cv_:

                self.cv_.wait_for(timeout=2*self.period_,
                                  predicate=
                                  lambda :
                                  (self._watermark_time() - self.t_prev_ >= self.period_)
                                  or
                                  rospy.is_shutdown())
                w = self._watermark_time()

            steps = 0

            while self.t_prev_ + self.period_ <= w and steps < MAX_CATCHUP_STEPS:

                # Advance the current time by the period
                self.t_curr_ = self.t_prev_ + self.period_



                self.t_prev_ = self.t_curr_
                steps = steps + 1

    def _getStateInterval(self, t_start, t_end) -> bool:
        if self.mocap_state_buffer_.empty():
            return False
        while self.mocap_state_buffer_.front()[0] <= t_start:
            self.mocap_state_buffer_.pop()
            if self.mocap_state_buffer_.empty():
                return False

    def _watermark_time(self):
        now = rospy.get_time()
        fresh_idxs = [i for i in range(len(self.time_latest_)) if self._freshByTTL(i, now)]

        if not fresh_idxs:
            return self.t_prev_
        return min(self.time_latest_[i] for i in fresh_idxs)

    def _freshByTTL(self, i, now_wall):
        return bool(now_wall - self.latest_tx_wall_[i])

    def _setup_rc_param(self):

        node_name = rospy.get_name()

        a_max = rospy.get_param(node_name + '/constraint/a_max')
        z_max = rospy.get_param(node_name + '/constraint/z_max')
        dpsi_dt_max = rospy.get_param(node_name + '/constraint/dpsi_dt_max')


        manualParam = {'a_max': a_max,
                       'z_max': z_max,
                       'dpsi_dt_max': dpsi_dt_max}

        return manualParam

    def _setup_gain_param(self):

        node_name = rospy.get_name()

        # Get translational control gain (PID)
        Kp_trans = rospy.get_param(node_name + '/pid/trans/Kp')
        Kd_trans = rospy.get_param(node_name + '/pid/trans/Kd')

        # Get rotational control gain (PD)
        Kp_ori = rospy.get_param(node_name + '/pid/ori/Kp')
        Kd_ori = rospy.get_param(node_name + '/pid/ori/Kd')

        # Get nominal dynamics parameter
        m = rospy.get_param(node_name + '/nominal_dynamics/m')
        J = rospy.get_param(node_name + '/nominal_dynamics/J')

        GainParam = {'Kp_trans': Kp_trans,
                     'Kd_trans': Kd_trans,
                     'Kp_ori': Kp_ori,
                     'Kd_ori': Kd_ori}

        DynParam = {'m': m,
                    'J': J}

        return GainParam, DynParam

    def _setup_drone_param(self):
        node_name = rospy.get_name()
        l = rospy.get_param(node_name + '/drone/arm_length')
        C_T = rospy.get_param(node_name + '/drone/rotor_const')
        K_m = rospy.get_param(node_name + '/drone/moment_const')
        T_min = rospy.get_param(node_name + '/drone/T_min')
        T_max = rospy.get_param(node_name + '/drone/T_max')

        droneParam = {'arm_length': l,
                      'rotor_const': C_T,
                      'moment_const': K_m,
                      'T_min': T_min,
                      'T_max': T_max}
        return droneParam