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

from manual_control.rc_converter import RcConverter, FlightMode
from manual_control.rc_controller import RcController
from utils.inverse_dynamics import InverseDynamics

from utils import quaternion_math
from geometry_msgs.msg import WrenchStamped

def rcIn_parsing(msg:RCIn):
    t_ros = rospy.get_rostime().to_sec()
    return (t_ros,
            np.array(object=msg.channels,dtype=np.int32))
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
                     msg.pose.pose.orientation.z,
                     msg.twist.twist.angular.x,
                     msg.twist.twist.angular.y,
                     msg.twist.twist.angular.z])

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

        p = np.zeros((3,))
        v = np.zeros((3,))
        q = np.array([1.0,0.0,0.0,0.0])
        w = np.zeros((3,))

        self.state = np.concatenate([p,v,q,w])

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
        self.RcConverter_obj = RcConverter(manualParam=manualParam)
        self.RcController_obj = RcController(GainParam=GainParam,
                                         DynParam=DynParam)
        self.InverseDynamics_obj = InverseDynamics(Param=droneParam)


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

        self.cmd_pub_timer = rospy.Timer(rospy.Duration(0.01),
                                         self._publishCallback)

        # Publisher
        self.cmd_pub = rospy.Publisher('/uav/cmd_raw',
                                       data_class=hexa_cmd_raw,
                                       queue_size=10)

        self.u_msg = hexa_cmd_raw()

        self.cv_ = threading.Condition()
        self.period_ = 0.010

        self.is_first_run = True

        self.t_curr_ = 0
        self.t_prev_ = self.t_curr_

        # RC, mocap state, wrench in turn
        self.time_latest_ = np.array([-1e3,
                                      -1e3,
                                      -1e3])
        self.latest_rx_wall_ = np.zeros((3,))
        self.time_out_ = np.array([0.015,
                                   0.015,
                                   0.015])

        self.process_thread = threading.Thread(target=self._process_func,
                                               daemon=True)
        self.process_thread.start()

    def _rcCallback(self, msg:RCIn):
        with self.cv_:
            rc_temp = rcIn_parsing(msg)
            if self.rc_in_buffer_.full():
                self.rc_in_buffer_.pop()
            self.rc_in_buffer_.push(rc_temp)

            self.time_latest_[0] = max(self.time_latest_[0], rc_temp[0])
            self.latest_rx_wall_[0] = rospy.get_time()
            self.cv_.notify_all()
    def _mocapCallback(self, msg:Odometry):
        with self.cv_:
            mocap_temp = odom_parsing(msg)
            if self.mocap_state_buffer_.full():
                self.mocap_state_buffer_.pop()
            self.mocap_state_buffer_.push(mocap_temp)

            self.time_latest_[1] = max(self.time_latest_[1], mocap_temp[0])
            self.latest_rx_wall_[1] = rospy.get_time()
            self.cv_.notify_all()

    def _wrenchCallback(self, msg:WrenchStamped):
        with self.cv_:
            wrench_temp = wrench_parsing(msg)
            if self.wrench_buffer_.full():
                self.wrench_buffer_.pop()
            self.wrench_buffer_.push(wrench_temp)
            self.time_latest_[2] = max(self.time_latest_[2], wrench_temp[0])
            self.latest_rx_wall_[2] = rospy.get_time()
            self.cv_.notify_all()

    def _publishCallback(self, event):
        self.cmd_pub.publish(self.u_msg)
    def _process_func(self):

        MAX_CATCHUP_STEPS = 3
        rospy.loginfo("Starting thread")

        while not rospy.is_shutdown():

            if self.is_first_run is True:
                self.t_curr_ = rospy.get_time()
                self.t_prev_ = self.t_curr_ - self.period_
                self.is_first_run = False

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

                t_ref = self.t_curr_ - 0.030

                rc_available = self._getBufferAfter(self.rc_in_buffer_,
                                                    t_ref)
                mocap_state_available = self._getBufferAfter(self.mocap_state_buffer_,
                                                             t_ref)
                wrench_available = self._getBufferAfter(self.wrench_buffer_,
                                                        t_ref)

                self._rc_update(rc_available, mocap_state_available, wrench_available)

                print('rc: ', self.rc_in_buffer_.size(),
                      'mocap: ', self.mocap_state_buffer_.size(),
                      'wrench: ', self.wrench_buffer_.size())

                self.t_prev_ = self.t_curr_
                steps = steps + 1

    def _rc_update(self, rc_available:bool,
                           mocap_state_available:bool,
                           wrench_available:bool):

        if rc_available and mocap_state_available and wrench_available:
            t_rc, rc_recent = self.rc_in_buffer_.back()
            self.RcConverter_obj.set_rc(rc_recent)
            a_des, z_des, dpsi_dt_des, mode = self.RcConverter_obj.get_rc_state()

            if mode == FlightMode.KILL:
                self._put_rpm_same_input(0)
            elif mode == FlightMode.ARMED:
                self._put_rpm_same_input(2000)
            elif mode == FlightMode.MANUAL_STAB:
                ref = np.concatenate([a_des, np.array([z_des, dpsi_dt_des])])
                state_curr = self.mocap_state_buffer_.front()[1:]
                tau_curr = self.wrench_buffer_.front()[4:]
                self.RcController_obj.set_ref_state(ref, state_curr, tau_curr)
                u = self.RcController_obj.get_control_input()
                rpm_des = self.InverseDynamics_obj.compute_des_rpm(u[0],u[1:])
                self._put_cmd_from_rpm_des(rpm_des)
        else:
            # RC io should be received.
            for i in range(6):
                self.u_msg.raw[i] = int(0)

    def _getBufferAfter(self, buffer, t_start) -> bool:
        if buffer.empty():
            return False
        while buffer.front()[0] < t_start:
            buffer.pop()
            if buffer.empty():
                return False
        return True

    def _put_rpm_same_input(self, u):
        for i in range(6):
            self.u_msg.raw[i] = int(u)

    def _put_cmd_from_rpm_des(self, rpm_des):
        for i in range(6):
            self.u_msg.raw[i] = int(rpm_des[i]*self.MaxBit/self.MaxRpm)

    def _watermark_time(self):
        now = rospy.get_time()
        fresh_idxs = [i for i in range(len(self.time_latest_)) if self._freshByTTL(i, now)]

        if not fresh_idxs:
            return self.t_prev_
        return min(self.time_latest_[i] for i in fresh_idxs)

    def _freshByTTL(self, i, now_wall):
        return bool(now_wall - self.latest_rx_wall_[i] <= self.time_out_[i])

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

if __name__ == '__main__':
    rospy.init_node('manual_control_node')
    node = ManualControlNode()
    rospy.spin()
    node.process_thread.join()