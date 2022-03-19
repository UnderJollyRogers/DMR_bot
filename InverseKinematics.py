# -*- coding: utf-8 -*-
"""
Inverse Kinematics of DMR_bot

@author: jsanjuan
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
def lambda2str(expr: function) -> str:
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
x, y, z = sp.symbols('x, y, z')
t = sp.symbols('t')
t1, t2, t3 = sp.symbols('t1, t2, t3')
Y = sp.Matrix([[t1, t2, t3]])

""" Denavit DH parameters """
T01 = homogeneousMatrix(  0,       0,  0, t1+sp.pi/2)
T12 = homogeneousMatrix(  0, sp.pi/2,  0, t2+sp.pi/2)
T23 = homogeneousMatrix( L1,       0,  0,         t3)
T34 = homogeneousMatrix(-L2,       0,  L3,          0)
T04 = T01*T12*T23*T34

""" Direct Kinematics """
directKinematics = sp.Matrix([T04[0,3], T04[1,3], T04[2,3]])
# list(directKinematics[0].free_symbols)
directKinematicsVariables = [L1, L2, L3, t1, t2, t3]
directKinematicsFunction = lambdify(directKinematicsVariables,
                                   directKinematics)

""" Solution T1 """
eq1 = sp.trigsimp((T04[0,3]-x)*cos(t1))
eq2 = sp.trigsimp((T04[1,3]-y)*sin(t1))
eq3 = sp.trigsimp(eq2 + eq1)
eq4 = eq3.subs([(cos(t1), (1-t**2)/(1+t**2)), (sin(t1), 2*t/(1+t**2))])
eq4 = sp.simplify(eq4*(1+t**2))
eq5 = sp.solve(eq4, t) 
theta1 = [2*sp.atan(eq5[0]), 2*sp.atan(eq5[1])]
theta1Variables = [L3, x, y]
theta1Function = lambdify(theta1Variables, theta1[0])

""" Solution T2 """
eq1 = sp.solve(sp.trigsimp(T04[2,3]) - z, cos(t2 + t3))[0]
eq2 = sp.solve(sp.trigsimp(T04[1,3]) - y, sin(t2 + t3))[0]
eq3 = eq1**2 + eq2**2 - 1
eq4 = sp.expand(eq3)
n,d = sp.fraction(eq4)
eq5 = sp.trigsimp(n)
eq5 = eq4.subs([(cos(t2), (1-t**2)/(1+t**2)), (sin(t2), 2*t/(1+t**2))])
eq6 = sp.simplify(eq5*(1+t**2))
n,d = sp.fraction(eq6)
eq7 = sp.solve(n, t)
theta2 = [2*sp.atan(eq7[2]), 2*sp.atan(eq7[3])]
theta2Variables = [L1, L2, L3, t1, y, z]
theta2Function = lambdify(theta2Variables, theta2)

""" Solution T3 """
theta3 = sp.solve(sp.trigsimp(T04[2,3]) - z, t3)
theta3Variables = [L1, L2, t2, z]
theta3Function = lambdify(theta3Variables, theta3)

""" Saving Inverse Kinematics """
output = "inverseKinematicsDMRbot.txt"
outfile = open(output, 'w')
str1 = lambda2str(theta1Function)
str2 = lambda2str(theta2Function)
str3 = lambda2str(theta3Function)
str4 = lambda2str(directKinematicsFunction)
print(str1, file=outfile)
print(str2, file=outfile)
print(str3, file=outfile)
print(str4, file=outfile)
outfile.close()
infile = open(output, "r")
for i in infile.readlines():
    print(i+"\n")
infile.close()