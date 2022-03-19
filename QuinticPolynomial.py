# -*- coding: utf-8 -*-
"""
Quintic polynomial

@author: jsanjuan
"""

import numpy as np

desiredPosition = -40
initialPosition = -50
deltaT = 10
deltaP = desiredPosition - initialPosition 
a = np.array([[deltaT**3,    deltaT**4,    deltaT**5], 
            [3*deltaT**2,  4*deltaT**3,  5*deltaT**4], 
               [6*deltaT, 12*deltaT**2, 20*deltaT**3]])
b = np.array([deltaP, 0, 0])
x = np.linalg.solve(a, b)
coefficients = np.concatenate((np.array([initialPosition, 0, 0]), x.T))
timePeriod = 1
time = np.arange(0, deltaT+timePeriod, timePeriod)
i = 1
# timeArray = np.array([1, time[i], time[i]**2, time[i]**3, time[i]**4, time[i]**5])
for i in range(len(time)):
    # print(time[i])
    timeArray = np.array([1, time[i], time[i]**2, time[i]**3, time[i]**4, time[i]**5])
    tempPosition = np.matmul(timeArray.T, coefficients)
    print(tempPosition)
    