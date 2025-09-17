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
from manual_control.rc_converter.rc_converter_alt import RcConverterAlt

# Three types of controller
from manual_control.rc_converter.rc_converter_alt import RcConverterAlt
from pos_control.pid_control import PosControl
from manual_control.rp_yrate_controller.rp_alt_controller import RpyzController

# Msg
from mavros_msgs.msg import RCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench
from nmpc_drone.msg import ref
from ros_libcanard.msg import hexa_cmd_raw

class control_node():

    def __init__(self):

        rospy.init_node('control_node')

        self.rc_state = np.zeros((12,))