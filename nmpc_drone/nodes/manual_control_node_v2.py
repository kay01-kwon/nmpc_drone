#! /usr/bin/env python3.8
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCIn
from mav_msgs.msg import Actuators
from manual_control.rc_converter.rc_converter_v2 import RcConverterV2
from manual_control.rp_yrate_controller.rp_vz_controller import RpVzController
from manual_control.rp_yrate_controller.inverse_dynamics import InverseDynamics
class manual_control_node_v2():
    def __init__(self):

        rospy.init_node('manual_control_v2', anonymous=True)

        self.rc_state = np.zeros((12,))
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.w = np.zeros((3,))

        self.rc_subsriber = []
        self.odom_subsriber = []
        self.cmd_pub = []

        self.RcConverter = RcConverterV2(ax_max = 1, ay_max = 1,
                                         vz_max = 0.3, psidot_max = 10)

        GainParam = {'Kv_trans': np.diag([0,0,100]),
                     'Kp_ori': np.diag([4,4,0.3]),
                     'Kd_ori': np.diag([1,1,0.01])}
        DynParam = {'m': 2.9,
                    'J': np.diag([0.052, 0.052, 0.070])}

        self.RpVzController = RpVzController(GainParam, DynParam)

        param = {'arm_length': 0.265,
                 'rotor_const': 1.33591e-05,
                'moment_const': 0.01569,
                'T_min': 0.100*9.81,
                'T_max': 0.90*9.81}

        self.InverseDynamics = InverseDynamics(param)

        self.u_msg = Actuators()

        self.ros_setup()

    def ros_setup(self):
        self.rc_subscriber = rospy.Subscriber('/mavros/rc/in',
                                              RCIn,
                                              self.rc_callback)
        self.odom_subscriber = rospy.Subscriber('/custom_hexacopter/ground_truth/odometry'
                                                , Odometry
                                                , self.odom_callback)
        self.cmd_pub = rospy.Publisher('/custom_hexacopter/command/motor_speed',
                                       Actuators,
                                       queue_size=10)

    def rc_callback(self, rc_msg):
        self.rc_state = rc_msg.channels
        self.RcConverter.set_throttle_rp_y_rate(self.rc_state)
        a_ref, v_ref, w_rate = self.RcConverter.get_avw_rate()

        self.RpVzController.set_rp_yrate(a_ref, v_ref, w_rate, self.q, self.w)
        u = self.RpVzController.compute_moment()

        rotor_speed = self.InverseDynamics.compute_des_rpm(u[0],u[1:])
        # print(rotor_speed)
        self.u_msg.angular_velocities = rotor_speed
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
    manual_control_obj = manual_control_node_v2()
    rospy.spin()