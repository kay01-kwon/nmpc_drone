import casadi as cs
import numpy as np
import os
import sys
from nmpc_pkg import tools
C = 2
u1 = cs.MX.sym('u1')
u2 = cs.MX.sym('u2')
u3 = cs.MX.sym('u3')
u4 = cs.MX.sym('u4')
u = cs.vertcat(u1, u2, u3, u4)

thrust = C*u**2

print(thrust)
print(thrust.shape)


q = cs.MX.sym('q',4)

test_mat = cs.MX.sym('R', 3, 3)

q_vec = tools.quat2quat_vec(q)

test_mat = tools.vec2skew_symm(q_vec)


print(cs.dot(q,q))