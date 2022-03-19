# -*- coding: utf-8 -*-
"""
This algorithm is used to compute the dynamics of DRMbot symbolically. 

This algorithm also generates dynamic matrices M, V, and G using the lambdify function and converting the
obtained function as a str file. 
"""

import sympy as sp
from sympy import Symbol
from sympy import cos
from sympy import sin
import base64
import cloudpickle
from sympy.utilities.lambdify import lambdify
from typing import TypeVar

function = TypeVar('Function')
def lambda2str(expr:function) -> str:
    b = cloudpickle.dumps(expr)
    s = base64.b64encode(b).decode()
    return s

def str2lambda(s:str) -> function:
    b = base64.b64decode(s)
    expr = cloudpickle.loads(b)
    return expr

def homogeneousMatrix(ai:Symbol, alphai:Symbol, di:Symbol, thetai:Symbol) -> Symbol:
    T = sp.Matrix([[            cos(thetai),            -sin(thetai),            0,              ai], \
                   [cos(alphai)*sin(thetai), cos(alphai)*cos(thetai), -sin(alphai), -di*sin(alphai)], \
                   [sin(alphai)*sin(thetai), sin(alphai)*cos(thetai),  cos(alphai),  di*cos(alphai)], \
                   [                      0,                       0,            0,               1]]) 
    return T

L1, L2, L3 = sp.symbols('L1, L2, L3')
t = sp.symbols('t')
t1 = sp.Function('t1')(t)
t2 = sp.Function('t2')(t)
t3 = sp.Function('t3')(t)
Y = sp.Matrix([[t1, t2, t3]])

""" Denavit DH parameters """
T01 = homogeneousMatrix(  0,       0,  0, t1+sp.pi/2)
T12 = homogeneousMatrix(  0, sp.pi/2,  0, t2+sp.pi/2)
T23 = homogeneousMatrix( L1,       0,  0,         t3)
T34 = homogeneousMatrix(-L2,       0,  L3,          0)
T04 = T01*T12*T23*T34
T04 = sp.trigsimp(T04)
dt1 = sp.diff(t1, t)
dt2 = sp.diff(t2, t)
dt3 = sp.diff(t3, t)
ddt1 = sp.diff(dt1, t)
ddt2 = sp.diff(dt2, t)
ddt3 = sp.diff(dt3, t)
dt = sp.Matrix([[dt1, dt2, dt3]])
dt = dt.T #.T := transpose of a matrix
ddt = sp.Matrix([[ddt1, ddt2, ddt3]])
ddt = ddt.T

""" Body 1 """
R01 = T01.col(range(0,3)).row(range(0,3))
R01_t = R01.T 
T02 = T01*T12
R02 = T02.col(range(0,3)).row(range(0,3))
R02_t = R02.T
T03 = T01*T12*T23
R03 = T03.col(range(0,3)).row(range(0,3))
R03_t = R03.T
Lcg1x = sp.symbols('Lcg1x')
Lcg1y = sp.symbols('Lcg1y')
Lcg1z = sp.symbols('Lcg1z')
centerOfGrav1 = sp.Matrix([[Lcg1x, Lcg1y, Lcg1z, 1]])
P1 = T02*centerOfGrav1.T 
P1 = P1.row(range(0,3))
J1 = P1.jacobian(Y)
A1_1 = J1*ddt
A1_2 = sp.diff(J1, t)*dt
A1 = A1_1 + A1_2 #Center of gravity acceleration
Wz_1 = sp.trigsimp(R02[0,0:3]*R02_t[0:3,1].jacobian(Y))
Wy_1 = sp.trigsimp(R02[2,0:3]*R02_t[0:3,0].jacobian(Y))
Wx_1 = sp.trigsimp(R02[1,0:3]*R02_t[0:3,2].jacobian(Y))
Jw1  = sp.Matrix([Wx_1, Wy_1, Wz_1])
alpha1_1 = Jw1*ddt 
alpha1_2 = sp.diff(Jw1, t)*dt
alpha1 = alpha1_1 + alpha1_2
m1, g = sp.symbols('m1, g')
F1 = -A1*m1 - sp.Matrix([[0],[0],[1]])*m1*g
T1 = sp.symbols('T1')
T2 = sp.symbols('T2')
T3 = sp.symbols('T3')
I1x, I1y, I1z = sp.symbols('I1x, I1y, I1z')
torque1 = R01[0:3,2]*T1 + R02[0:3,2]*T2 - R02*sp.Matrix([[I1x, 0, 0],[0, I1y, 0],[0, 0, I1z]])*R02_t*alpha1 - R03[0:3,2]*T3

""" Body 2 """
Lcg2x = sp.symbols('Lcg2x')
Lcg2y = sp.symbols('Lcg2y')
Lcg2z = sp.symbols('Lcg2z')
centerOfGrav2 = sp.Matrix([[Lcg2x, Lcg2y, Lcg2z, 1]])
P2 = T03*centerOfGrav2.T 
P2 = sp.simplify(P2)
P2 = P2.row(range(0,3))
J2 = P2.jacobian(Y)
A2_1 = J2*ddt
A2_2 = sp.diff(J2, t)*dt
A2 = A2_1 + A2_2
Wz_2 = sp.trigsimp(R03[0,0:3]*R03_t[0:3,1].jacobian(Y))
Wy_2 = sp.trigsimp(R03[2,0:3]*R03_t[0:3,0].jacobian(Y))
Wx_2 = sp.trigsimp(R03[1,0:3]*R03_t[0:3,2].jacobian(Y))
Jw2  = sp.Matrix([Wx_2, Wy_2, Wz_2])
alpha2_1 = Jw2*ddt
alpha2_2 = sp.diff(Jw2, t)*dt
alpha2 = alpha2_1 + alpha2_2
m2 = sp.symbols('m2')
F2 = -A2*m2 - sp.Matrix([[0],[0],[1]])*m2*g
I2x, I2y, I2z = sp.symbols('I2x, I2y, I2z')
torque2 = R03[0:3,2]*T3 - R02*sp.Matrix([[I2x, 0, 0],[0, I2y, 0],[0, 0, I2z]])*R02_t*alpha2

""" Assembly of the equations """
dynamicEquation = Jw1.T*torque1 + J1.T*F1 + Jw2.T*torque2 + J2.T*F2
dt1, dt2, dt3 = sp.symbols('dt1, dt2, dt3')
ddt1, ddt2, ddt3 = sp.symbols('ddt1, ddt2, ddt3')
dynamicEquation = dynamicEquation.subs([(sp.Derivative(t1, (t, 2)), ddt1), (sp.Derivative(t2, (t, 2)), ddt2), (sp.Derivative(t3, (t, 2)), ddt3),
                                        (sp.Derivative(t1, t), dt1), (sp.Derivative(t2, t), dt2), (sp.Derivative(t3, t), dt3)])

t1, t2, t3 = sp.symbols('t1, t2, t3')
dynamicEquation = dynamicEquation.subs([(sp.Function('t1')(t), t1),
                                        (sp.Function('t2')(t), t2), 
                                        (sp.Function('t3')(t), t3)])
dynamicEquation = sp.simplify(dynamicEquation)
""" Computation of matrices M, V, and G """
T = sp.solve(tuple(dynamicEquation), T1, T2, T3)
T = sp.Matrix([[T[T1]], \
               [T[T2]], \
               [T[T3]]])
G = g*sp.Matrix([[T[0].coeff(g, 1)], \
                 [T[1].coeff(g, 1)], \
                 [T[2].coeff(g, 1)]])
dynamicEquation2 = T - G
M = sp.Matrix([[dynamicEquation2[0].coeff(ddt1, 1), dynamicEquation2[0].coeff(ddt2, 1), dynamicEquation2[0].coeff(ddt3, 1)], \
               [dynamicEquation2[1].coeff(ddt1, 1), dynamicEquation2[1].coeff(ddt2, 1), dynamicEquation2[1].coeff(ddt3, 1)],
               [dynamicEquation2[2].coeff(ddt1, 1), dynamicEquation2[2].coeff(ddt2, 1), dynamicEquation2[2].coeff(ddt3, 1)]])
ddt = sp.Matrix([[ddt1],[ddt2],[ddt3]])
V   = sp.simplify(dynamicEquation2 - M*ddt)

""" Lambdification of matrices """

"""Matrix M;

Note that this matrix requires the following variables: 
    (I1x, I1y, I1z,
     I2x, I2y, I2z,
     L1,
     Lcg1x, Lcg1y, Lcg1z,
     Lcg2x, Lcg2y, Lcg2z,
     m1, m2,
     t2, t3)"""
    
varM =  [I1x, I1y, I1z,
         I2x, I2y, I2z,
          L1,
       Lcg1x, Lcg1y, Lcg1z,
       Lcg2x, Lcg2y, Lcg2z,
          m1, m2,
          t2, t3]
solM = lambdify(varM, M)

"""Matrix V;

Note that this matrix requries the following variables:
    (I1x, I1y, I1z,
     I2x, I2y, I2z,
     L1,
     Lcg1x, Lcg1y, Lcg1z,
     Lcg2x, Lcg2y, Lcg2z,
     m1, m2,
     dt1, dt2, dt3,
     t2, t3)
"""
varV = [I1x, I1y, I1z,
        I2x, I2y, I2z,
        L1,
        Lcg1x, Lcg1y, Lcg1z,    
        Lcg2x, Lcg2y, Lcg2z,
        m1, m2,
        dt1, dt2, dt3,
        t2, t3]
solV = lambdify(varM, V)

"""Matrix G;

Note that this matrix requries the following variables:
    (L1, 
     Lcg1x, Lcg1y,
     Lcg2x, Lcg2y, 
     g, 
     m1, m2, 
     t2, t3)
"""
varG = [L1, 
        Lcg1x, Lcg1y,
        Lcg2x, Lcg2y, 
        g, 
        m1, m2, 
        t2, t3]
solG = lambdify(varG, G)

""" Saving matrices """
output = "dynamicMatrices.txt"
outfile = open(output, 'w')
str1 = lambda2str(solM)
str2 = lambda2str(solV)
str3 = lambda2str(solG)
print(str1, file=outfile)
print(str2, file=outfile)
print(str3, file=outfile)
outfile.close()
infile = open(output,"r")
for i in infile.readlines():
    print(i+"\n")
infile.close()
