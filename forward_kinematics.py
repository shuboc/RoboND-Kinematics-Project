import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

# q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
# d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
# a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
# alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

q = symbols('q0:8')
d = symbols('d0:8')
a = symbols('a0:7')
alpha = symbols('alpha0:7')

s = {alpha0: 0,     a0: 0,      d1: 0.75,
     alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,
     alpha2: 0,     a2: 1.25,   d3: 0,
     alpha3: -pi/2, a3: -0.054, d4: 1.5,
     alpha4: pi/2,  a4: 0,      d5: 0,
     alpha5: -pi/2, a5: 0,      d6: 0,
     alpha6: 0,     a6: 0,      d7: 0.303, q7: 0
}

# T0_1 = Matrix([[cos(q1),                        -sin(q1),            0,              a0],
#                [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
#                [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
#                [                  0,                   0,            0,               1]])
# T0_1 = T0_1.subs(s)

T0_1 = Matrix([[cos(q[1]),                            -sin(q[1]),              0,                a[0]],
               [sin(q[1])*cos(alpha[0]), cos(q[1])*cos(alpha[0]), -sin(alpha[0]), -sin(alpha[0])*d[1]],
               [sin(q[1])*sin(alpha[0]), cos(q[1])*sin(alpha[0]),  cos(alpha[0]),  cos(alpha[0])*d[1]],
               [                      0,                       0,              0,                   1]])
T0_1 = T0_1.subs(s)

# T1_2 = Matrix([[cos(q2),                        -sin(q2),            1,              a1],
#                [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
#                [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
#                [                  0,                   0,            0,               1]])
# T1_2 = T1_2.subs(s)

print("T0_1 = ", T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
