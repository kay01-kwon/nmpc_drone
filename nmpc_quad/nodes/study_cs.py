import casadi as cs
import numpy as np
import os
import sys
import shutil
'''
Append nmpc_pkg directory using sys module
'''
dir_path = os.path.dirname(os.path.realpath(__file__))
print(dir_path)

pkg_dir = dir_path + '/nmpc_pkg'
print(pkg_dir)
sys.path.append(dir_path + '/nmpc_pkg')

curr_dir = os.getcwd()
prev_dir = os.path.dirname(curr_dir)
prev_dir = os.path.dirname(prev_dir)
print(prev_dir)
c_gen_dir = prev_dir + '/c_generated_code'
acados_json_dir = prev_dir + '/acados_json'
print(c_gen_dir)

if os.path.exists(c_gen_dir):
    print('Remove c_generated_code')
    shutil.rmtree(c_gen_dir)

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
# q1 = cs.MX.sym('q1',4)
# q2 = cs.MX.sym('q2',4)
#
# test_mat = cs.MX.sym('R', 3, 3)
#
# q_vec = tools.quat2quat_vec(q)
#
# test_mat = tools.vec2skew_symm(q_vec)

# x_ref = np.zeros((13,))
# u_ref = np.zeros((4,))

# y_ref = np.concatenate((x_ref,u_ref))

# print(y_ref)
#
# q1_L = cs.vertcat(
#     cs.horzcat(q1[3], -q1[0], -q1[2], -q1[3]),
#     cs.horzcat(q1[0], q1[3], -q1[2], q1[1]),
#     cs.horzcat(q1[1], q1[3], q1[3], -q1[0]),
#     cs.horzcat(q1[2], -q1[1], q1[0], q1[3])
# )
#
# print(cs.mtimes(q1_L,q2))
#
# state = np.zeros((3,))
#
# state[0] = 1
# state[1] = 2
# state[2] = 3
#
# print("state",state[:3])

Q = np.array([0.5, 0.5, 0.5,
            0.05, 0.05, 0.05,
            0.1, 0.1, 0.1,
            0.01, 0.01, 0.01])

q_mask = np.array([1,1,1,1,1,1,1,1,1,1,1,1])

q_diagonal = np.concatenate((Q[:3],np.mean(Q[3:6])[np.newaxis],Q[3:]))

q_mask = np.concatenate((q_mask[:3],np.zeros(1), q_mask[3:]))

print(q_mask)

q_diagonal *= q_mask

print(q_diagonal)
