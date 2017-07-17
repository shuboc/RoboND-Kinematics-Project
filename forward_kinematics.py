import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

s = {alpha0: 0,     a0: 0,      d1: 0.75,
     alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,
     alpha2: 0,     a2: 1.25,   d3: 0,
     alpha3: -pi/2, a3: -0.054, d4: 1.5,
     alpha4: pi/2,  a4: 0,      d5: 0,
     alpha5: -pi/2, a5: 0,      d6: 0,
     alpha6: 0,     a6: 0,      d7: 0.303, q7: 0
}

def create_ht_from_dh_params(alpha, a, d, q):
    return Matrix([[cos(q),                      -sin(q),           0,             a],
                   [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                0,                 0,           0,             1]])

T0_1 = create_ht_from_dh_params(alpha0, a0, d1, q1)
T0_1 = T0_1.subs(s)

T1_2 = create_ht_from_dh_params(alpha1, a1, d2, q2)
T1_2 = T1_2.subs(s)

T2_3 = create_ht_from_dh_params(alpha2, a2, d3, q3)
T2_3 = T2_3.subs(s)

T3_4 = create_ht_from_dh_params(alpha3, a3, d4, q4)
T3_4 = T3_4.subs(s)

T4_5 = create_ht_from_dh_params(alpha4, a4, d5, q5)
T4_5 = T4_5.subs(s)

T5_6 = create_ht_from_dh_params(alpha5, a5, d6, q6)
T5_6 = T5_6.subs(s)

T6_G = create_ht_from_dh_params(alpha6, a6, d7, q7)
T6_G = T6_G.subs(s)

print("T0_1 = ", T0_1)
print("T1_2 = ", T1_2)
print("T2_3 = ", T2_3)
print("T3_4 = ", T3_4)
print("T4_5 = ", T4_5)
print("T5_6 = ", T5_6)
print("T6_G = ", T6_G)

T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

# correction term
R_z = Matrix([[cos(np.pi), -sin(np.pi), 0, 0],
              [sin(np.pi),  cos(np.pi), 0, 0],
              [         0,           0, 1, 0],
              [         0,           0, 0, 1]])

R_y = Matrix([[ cos(-np.pi/2), 0, sin(-np.pi/2), 0],
              [             0, 1,             0, 0],
              [-sin(-np.pi/2), 0, cos(-np.pi/2), 0],
              [             0, 0,             0, 1]])

R_corr = R_z * R_y

print("R_corr = ", R_corr)

T_total = T0_G * R_corr

T_total = T_total.evalf(subs={q1: -0.63, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})

print("T_total = ", T_total)

'''
R = T_total[0:3, 0:3]

print("R = ", R)

roll = atan2(R[1,0], R[0,0])
pitch = atan2(-R[2,0], sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0]))
yaw = atan2(R[2,1], R[2,2])

print 'roll:', roll
print 'pitch:', pitch
print 'yaw:', yaw
'''

'''
def rotate_x(angle):
	return Matrix([[ 1,              0,        0],
	       [ 0,        cos(angle), -sin(angle)],
	       [ 0,        sin(angle),  cos(angle)]])

def rotate_y(angle):
	return Matrix([[ cos(angle), 0, sin(angle)],
	       [          0, 1,          0],
	       [-sin(angle), 0, cos(angle)]])

def rotate_z(angle):
	return Matrix([[cos(angle), -sin(angle), 0],
	       [sin(angle),  cos(angle), 0],
	       [         0,           0, 1]])

R_total = T_total[0:3, 0:3]
R0_G = R_total * rotate_y(pi/2)* rotate_z(-pi) 

print T0_G[0:3, 0:3].evalf(subs={q1: -0.63, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})
print R0_G
'''
