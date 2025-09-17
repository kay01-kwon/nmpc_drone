#! /usr/bin/env python3.8

import numpy as np
from queue import Queue

import rospy
import threading

from nmpc_drone.msg import ref
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from mavros_msgs.msg import RCIn

def ref_parsing(msg):
    return np.array([msg.header.stamp.to_sec(),
                     msg.p_des[:],
                     msg.psi_des,
                     msg.v_des[:],
                     msg.dpsi_des])

def odom_parsing(msg):
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

def wrenchStamped_parsing(msg):
    return np.array([msg.header.stamp.to_sec(),
                     msg.wrench.force.x,
                     msg.wrench.force.y,
                     msg.wrench.force.z,
                     msg.wrench.torque.x,
                     msg.wrench.torque.y,
                     msg.wrench.torque.z])

def rcIn_parsing(msg):
    return np.array([msg.header.stamp.to_sec(),
                     msg.channels])
class RosSyncExample():
    def __init__(self):

        self.ref_buffer_ = Queue(maxsize=10)
        self.state_buffer_ = Queue(maxsize=10)
        self.wrench_buffer_ = Queue(maxsize=10)
        self.rc_in_buffer_ = Queue(maxsize=10)

        self.ref_subscriber = rospy.Subscriber('/ref',
                                               data_class=ref,
                                               callback=self.refCallback,
                                               queue_size=10,
                                               tcp_nodelay=True)

        self.state_subscriber = rospy.Subscriber('/custom_hexacopter/ground_truth/odometry',
                                                 data_class=Odometry,
                                                 callback=self.odomCallback,
                                                 queue_size=10,
                                                 tcp_nodelay=True)

        self.wrench_subscriber = rospy.Subscriber('/hgdo/wrench',
                                                  data_class=WrenchStamped,
                                                  callback=self.wrenchCallback,
                                                  queue_size=10,
                                                  tcp_nodelay=True)

        self.rcIn_subscriber = rospy.Subscriber('/rc/in',
                                                data_class=RCIn,
                                                callback=self.rcInCallback,
                                                queue_size=10,
                                                tcp_nodelay=True)

        self.cv_ = threading.Condition()
        self.period_ = 0.010

        self.t_curr_  = rospy.get_time()
        self.t_prev_ = self.t_curr_

        self.time_latest_ = np.array([-1e3, -1e3, -1e3, -1e3])
        self.last_rx_wall_ = np.zeros((4,))
        self.time_out_ = np.array([0.015, 0.015, 0.015, 0.015])
        self.aggregate_thread_ = threading.Thread(target=self.aggregate,
                                                  name='aggregate_thread',
                                                  daemon=True)
        self.aggregate_thread_.start()

    def refCallback(self, msg):
        with self.cv_:
            ref_temp = ref_parsing(msg)

            if self.ref_buffer_.full():
                self.ref_buffer_.get()
                self.ref_buffer_.put(ref_temp)
            else:
                self.ref_buffer_.put(ref_temp)

            self.time_latest_[0] = max(self.time_latest_[0], ref_temp[0])
            self.last_rx_wall_[0] = rospy.get_time()
            self.cv_.notify_all()

    def odomCallback(self, msg):
        with self.cv_:
            odom_temp = odom_parsing(msg)
            if self.state_buffer_.full():
                self.state_buffer_.get()
                self.state_buffer_.put(odom_temp)
            else:
                self.state_buffer_.put(odom_temp)

            self.time_latest_[1] = max(self.time_latest_[1], odom_temp[0])
            self.last_rx_wall_[1] = rospy.get_time()
            self.cv_.notify_all()

    def wrenchCallback(self, msg):
        with self.cv_:
            wrench_temp = wrenchStamped_parsing(msg)
            if self.wrench_buffer_.full():
                self.wrench_buffer_.get()
                self.wrench_buffer_.put(wrench_temp)
            else:
                self.wrench_buffer_.put(wrench_temp)
            self.time_latest_[2] = max(self.time_latest_[2], wrench_temp[0])
            self.last_rx_wall_[2] = rospy.get_time()
            self.cv_.notify_all()

    def rcInCallback(self, msg):
        with self.cv_:
            rcIn_temp = rcIn_parsing(msg)
            if self.rc_in_buffer_.full():
                self.rc_in_buffer_.get()
                self.rc_in_buffer_.put(rcIn_temp)
            else:
                self.rc_in_buffer_.put(rcIn_temp)
            self.time_latest_[3] = max(self.time_latest_[3], rcIn_temp[0])
            self.last_rx_wall_[3] = rospy.get_time()
            self.cv_.notify_all()

    def aggregate(self):

        t_loop = 0.0
        steps = 0
        MAX_CATCHUP_STEPS = 3
        rospy.loginfo("Starting aggregate thread")
        while not rospy.is_shutdown():

            steps = 0
            with self.cv_:
                self.cv_.wait_for(timeout=2*self.period_,
                              predicate =
                              lambda : (self.watermark_time() - self.t_prev_ >= self.period_)
                              or rospy.is_shutdown())
                w = self.watermark_time()

            while self.t_prev_ + self.period_ <= w and steps < MAX_CATCHUP_STEPS:

                self.t_curr_ = self.t_prev_ + self.period_

                t_loop = self.t_curr_ - self.t_prev_
                if t_loop >= 11:
                    rospy.loginfo('loop time: %.3f', t_loop*1000.0)

                self.t_prev_ = self.t_curr_
                steps = steps + 1

    def watermark_time(self):

        now = rospy.get_time()

        fresh_idxs = [i for i in range(len(self.time_latest_)) if self.freshByTTL(i, now)]

        if not fresh_idxs:
            return self.t_prev_
        return min(self.time_latest_[i] for i in fresh_idxs)

    def freshByTTL(self, i, now_wall):
        return bool(now_wall - self.last_rx_wall_[i] <= self.time_out_[i])

if __name__ == '__main__':
    rospy.init_node('ros_sync_example')
    ros_sync_example = RosSyncExample()
    rospy.spin()
    ros_sync_example.aggregate_thread_.join()