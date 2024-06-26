import casadi as cs
import numpy as np
import os
import sys
from nmpc_pkg import tools
# C = 2
# u1 = cs.MX.sym('u1')
# u2 = cs.MX.sym('u2')
# u3 = cs.MX.sym('u3')
# u4 = cs.MX.sym('u4')
# u = cs.vertcat(u1, u2, u3, u4)
#
# thrust = C*u**2
#
# print(thrust)
# print(thrust.shape)
#
#
q1 = cs.MX.sym('q1',4)
q2 = cs.MX.sym('q2',4)
#
# test_mat = cs.MX.sym('R', 3, 3)
#
# q_vec = tools.quat2quat_vec(q)
#
# test_mat = tools.vec2skew_symm(q_vec)

x_ref = np.zeros((13,))
u_ref = np.zeros((4,))

y_ref = np.concatenate((x_ref,u_ref))

print(y_ref)

q1_L = cs.vertcat(
    cs.horzcat(q1[3], -q1[0], -q1[2], -q1[3]),
    cs.horzcat(q1[0], q1[3], -q1[2], q1[1]),
    cs.horzcat(q1[1], q1[3], q1[3], -q1[0]),
    cs.horzcat(q1[2], -q1[1], q1[0], q1[3])
)

print(cs.mtimes(q1_L,q2))

