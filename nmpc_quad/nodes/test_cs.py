import casadi as cs
import numpy as np

C = 2
u1 = cs.MX.sym('u1')
u2 = cs.MX.sym('u2')
u3 = cs.MX.sym('u3')
u4 = cs.MX.sym('u4')
u = cs.vertcat(u1, u2, u3, u4)

thrust = C*u**2

print(thrust)
print(thrust.shape)