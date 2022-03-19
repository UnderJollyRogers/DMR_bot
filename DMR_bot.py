# -*- coding: utf-8 -*-
"""
DMR_bot class

@author: jsanjuan
"""
import numpy as np
import Motor as Motor
import base64
import cloudpickle
import time
from typing import TypeVar

function = TypeVar('Function')
def str2Lambda(s: str) -> function:
    b = base64.b64decode(s)
    expr = cloudpickle.loads(b)
    return expr

class DMR_bot:
    def __init__(self) -> None:
        self.joint1 = Motor.Motor(3, 7200, 100, 0.42723)
        self.joint2 = Motor.Motor(2, 4800, 100, 0.118449)
        self.joint3 = Motor.Motor(1, 4800, 100, 0.118449)
        self._L1 = 380 #mm
        self._L2 = 260 #mm
        self._L3 = 180 #mm
        self._Lcg1x = 11*25.4/1000
        self._Lcg1y = 0
        self._Lcg2x = -8*25.4/1000
        self._Lcg2y = 0
        self._g = 9.81
        self._m1 = 0.703
        self._m2 = 0.324
        openFunction = "inverseKinematicsDMRbot.txt"
        infile = open(openFunction, "r")        
        strSolution = ['','','','']
        count = 0
        for i in infile.readlines():
            strSolution[count] = i
            count = count + 1
        infile.close()
        self._solTheta1 = str2Lambda(strSolution[0]) #(L3, x, y)
        self._solTheta2 = str2Lambda(strSolution[1]) #(L1, L2, L3, t1, y, z)
        self._solTheta3 = str2Lambda(strSolution[2]) #(L1, L2, t2, z)
        self._directKinematics = str2Lambda(strSolution[3]) #(L1, L2, L3, t1, t2, t3)
        openFunction = "dynamicMatrices.txt"
        infile = open(openFunction, "r")    
        strDynamicSolution  = ['','','']
        count = 0
        for i in infile.readlines():
            strDynamicSolution[count] = i
            count = count + 1
        infile.close()
        self._matrixM = str2Lambda(strDynamicSolution[0])
        self._matrixV = str2Lambda(strDynamicSolution[1])
        self._matrixG = str2Lambda(strDynamicSolution[2])
        self.actualJoint1  = None
        self.actualJoint2  = None
        self.actualJoint3  = None
        self.desiredJoint1 = None
        self.desiredJoint2 = None
        self.desiredJoint3 = None
        self._desiredPosition = None
        self._actualPosition = None
        self._isEnable = False
        self._dictionaryOfOperations = {0:"idleState",
                                        1:"robotPositionControl",
                                        2:"robotSynchronousPositionControl",
                                        3:"initialPosition",
                                        4:"joystickMode",
                                        5:"teachingMode"}
        
        self._dictionaryOfMotorConfigurations = {0:"idleState",
                                                 1:"robotPositionControl",
                                                 2:"robotSynchronousPositionControl",
                                                 3:"teachingMode"}
        self._modeOfoperation = 0
        self._motorConfiguration = 0
        self._timePeriod = 0.001/2

    """ Properties Getter """
    @property 
    def L1(self) -> float:
        return self._L1
    
    @property
    def L2(self) -> float:
        return self._L2
    
    @property
    def L3(self) -> float:
        return self._L3
    
    @property 
    def solTheta1(self) -> function:
        return self._solTheta1
    
    @property
    def solTheta2(self) -> function:
        return self._solTheta2
    
    @property
    def solTheta3(self) -> function:
        return self._solTheta3
    
    @property 
    def directKinematics(self) -> function:
        return self._directKinematics
    
    @property 
    def actualPosition(self) -> None:
        self.__getJointPosition()
        return [self._actualPosition[0][0], 
                self._actualPosition[1][0], 
                self._actualPosition[2][0]]
    
    @property 
    def desiredPosition(self) -> list:
        if any(self._desiredPosition) is None:
            print("No desired position assigned")
        else:
            return [self._desiredPosition[0][0],
                    self._desiredPosition[1][0],
                    self._desiredPosition[2][0]]
    
    @property
    def motorConfiguration(self) -> None:
        return self._dictionaryOfMotorConfigurations[self._motorConfiguration]
    
    @property
    def modeOfOperation(self) -> None:
        return self._dictionaryOfOperations[self._modeOfOperation]

  
    """ Properties Setter """
    @modeOfOperation.setter
    def modeOfOperation(self, mode:int) -> None:
        self._modeOfOperation = mode
    
    @motorConfiguration.setter
    def configureMotor(self, mode:int) -> None:
        self._motorConfiguration = mode
        
    """ Decoration Shelf """    
    def _verifyConfigurationDecorator(instruction:function) -> function:
        def checkConfiguration(self, *args):
            instruction(self, *args)
            if self.modeOfOperation == "robotPositionControl" :
                if self.motorConfiguration != "robotPositionControl":
                    self.__enableRobotPositionControl()
                self.__getJointPosition()
                self.__inverseKinematics(args[0], args[1], args[2])
                length = np.linalg.norm(self._desiredPosition - self._actualPosition)
                t = length/args[3]
                velJoint1 = np.abs(self.desiredJoint1 - self.actualJoint1)/t
                velJoint2 = np.abs(self.desiredJoint2 - self.actualJoint2)/t
                velJoint3 = np.abs(self.desiredJoint3 - self.actualJoint3)/t
                self.joint1.positionControl(self.desiredJoint1, velJoint1)
                self.joint2.positionControl(-1*self.desiredJoint2, velJoint2)
                self.joint3.positionControl(-1*self.desiredJoint3, velJoint3)
                t0 = time.time()
                while True:
                    self.__getJointPosition()
                    positionChecker = np.linalg.norm(self._desiredPosition - self._actualPosition)
                    t1 = time.time()
                    if positionChecker < 1 or (t1-t0)>t:
                        self.showInfo()
                        return 
            if self.modeOfOperation == "robotSynchronousPositionControl" or self.modeOfOperation == "initialPosition" :
                if self.motorConfiguration != "robotSynchronousPositionControl":
                    self.__enableRobotSynchronousPositionControl()
                self.__getJointPosition()
                if self.modeOfOperation == "initialPosition":
                    print("Going to initial position")
                    delta = max([self.actualJoint1, self.actualJoint2, self.actualJoint3])
                    self.desiredJoint1 = 0
                    self.desiredJoint2 = 0
                    self.desiredJoint3 = 0
                    t = delta/10
                else:
                    self.__inverseKinematics(args[0], args[1], args[2])
                    length = np.linalg.norm(self._desiredPosition - self._actualPosition)
                    t = length/args[3]
                coefficientsJoint1 = self.__quinticCoefficients(self.actualJoint1, self.desiredJoint1, t)
                print(coefficientsJoint1)
                coefficientsJoint2 = self.__quinticCoefficients(self.actualJoint2, self.desiredJoint2, t)
                print(coefficientsJoint2)
                coefficientsJoint3 = self.__quinticCoefficients(self.actualJoint3, self.desiredJoint3, t)
                print(coefficientsJoint3)
                timeMatrix = np.arange(0, t + self._timePeriod, self._timePeriod*2)
                for i in range(len(timeMatrix)):
                    timeArray = np.array([1, timeMatrix[i], timeMatrix[i]**2, timeMatrix[i]**3, timeMatrix[i]**4, timeMatrix[i]**5])
                    temporalPositionJoint1 = np.matmul(timeArray.T, coefficientsJoint1)
                    temporalPositionJoint2 = np.matmul(timeArray.T, coefficientsJoint2)
                    temporalPositionJoint3 = np.matmul(timeArray.T, coefficientsJoint3)
                    self.joint1.synchronousPositionControl(temporalPositionJoint1, 0)
                    self.joint2.synchronousPositionControl(-1*temporalPositionJoint2, 0)
                    self.joint3.synchronousPositionControl(-1*temporalPositionJoint3, 0)
                    initialTime = time.time()
                    while True:
                        finalTime = time.time()
                        if finalTime - initialTime > self._timePeriod:
                            # print(finalTime - initialTime)
                            break
                print("Initial position reached!")
            if self.modeOfOperation == "joystickMode":
                if self.motorConfiguration != "robotSynchronousPositionControl":
                    self.__enableRobotSynchronousPositionControl()
                self.__inverseKinematics(args[0], args[1], args[2])
                self.joint1.synchronousPositionControl(self.desiredJoint1, 0)
                self.joint2.synchronousPositionControl(-1*self.desiredJoint1, 0)
                self.joint3.synchronousPositionControl(-1*self.desiredJoint1, 0)
                initialTime = time.time()
                while True:
                    finalTime = time.time()
                    if finalTime - initialTime > self._timePeriod:
                        # print(finalTime - initialTime)
                        break
            if self.modeOfOperation == "teachingMode":
                if self.motorConfiguration != "teachingMode":
                    self.__enableTeachingMode()
                try:
                    self.__getJointPosition()
                    aJ1 = self.actualJoint1
                    # aJ2 = self.actualJoint2
                    # aJ3 = self.actualJoint3
                    while True:
                        self.__getJointPosition()   
                        if aJ1 != self.actualJoint1:
                            self.joint1.synchronousPositionControl(self.actualJoint1, -1*np.sign(aJ1 - self.actualJoint1)*5)
                            aJ1 = self.actualJoint1  
                        T1 = 0.703*9.81*11*25.4/1000*np.cos(np.deg2rad(90 + self.actualJoint2)) + \
                             0.324*9.81*(self.L1/1000*np.cos(np.deg2rad(90 + self.actualJoint2)) + \
                                 8*25.4/1000*np.cos(np.deg2rad(-90 + self.actualJoint3 - self.actualJoint2)))
                        T2 = 0.324*9.81*8*25.4/1000*np.cos(np.deg2rad(-90 + self.actualJoint3 + self.actualJoint2))
                        print("Jr1="+str(self.actualJoint2)+" Jr2="+str(self.actualJoint3))
                        print("Ja1="+str(90 + self.actualJoint2)+" Ja2="+str(-90 + self.actualJoint3 + self.actualJoint2))
                        # print(-90 + self.actualJoint3 - self.actualJoint2)
                        self.joint2.torqueControl(-T1)
                        self.joint3.torqueControl(-T2)
                except KeyboardInterrupt:
                    pass
            return
        return checkConfiguration
    
    """ Public Methods """
    def disconnect(self):
        Motor.Motor.disconnect()
    
    @_verifyConfigurationDecorator
    def robotPositionControl(self, x:float, y:float, z:float, linearVel:float) -> None:
        self.modeOfOperation = 1
    
    @_verifyConfigurationDecorator
    def robotSynchronousPositionControl(self, x:float, y:float, z:float, linearVel:float) -> None:
        self.modeOfOperation = 2
    
    @_verifyConfigurationDecorator
    def initialPosition(self):
        self.modeOfOperation = 3
        
    @_verifyConfigurationDecorator
    def joystickMode(self, x:float, y:float, z:float) -> None:
        self.modeOfOperation = 4    
    
    @_verifyConfigurationDecorator
    def teachingMode(self) -> None:
        self.modeOfOperation = 5
        
    """ private Methods """
    def __enableRobotPositionControl(self) -> None:
        self.joint1.enablePositionControl()
        self.joint2.enablePositionControl()
        self.joint3.enablePositionControl()
        self.configureMotor = 1
       
    def __enableRobotSynchronousPositionControl(self) -> None:
        self.joint1.enableSynchronousPositionControl()
        self.joint2.enableSynchronousPositionControl()
        self.joint3.enableSynchronousPositionControl()
        self.configureMotor = 2
    
    def __enableTeachingMode(self) -> None:
        self.joint1.enableSynchronousPositionControl()
        self.joint2.enableTorqueControl()
        self.joint3.enableTorqueControl()
        self.configureMotor = 3
        
    def __getJointPosition(self) -> None:
        self.actualJoint1 = self.joint1.position
        self.actualJoint2 = -1*self.joint2.position
        self.actualJoint3 = -1*self.joint3.position
        self._actualPosition = self.directKinematics(self.L1, self.L2, self.L3,
                                                         self.actualJoint1*np.pi/180, 
                                                         self.actualJoint2*np.pi/180,
                                                         self.actualJoint3*np.pi/180) #Joint position in radians
  
    def __quinticCoefficients(self, initialPosition:float, desiredPosition:float, deltaT: float) -> np.array:
        deltaP = desiredPosition - initialPosition 
        if deltaP != 0:
            a = np.array([[deltaT**3,    deltaT**4,    deltaT**5], 
                        [3*deltaT**2,  4*deltaT**3,  5*deltaT**4], 
                           [6*deltaT, 12*deltaT**2, 20*deltaT**3]])
            b = np.array([deltaP, 0, 0])
            x = np.linalg.solve(a, b)
            coefficients = np.concatenate((np.array([initialPosition, 0, 0]), x.T))
        else:
            coefficients = np.array([0,0,0,0,0,0])
        return coefficients
    
    def __inverseKinematics(self, x:float, y:float, z:float) -> None:
        desiredPosition = np.array([[x],[y],[z]])
        desiredJoint1 = self.solTheta1(self.L3, x, y)
        Theta2 = self.solTheta2(self.L1, self.L2, self.L3, 
                                desiredJoint1, y, z)
        p1 = [self.L1*np.cos(Theta2[0]), self.L1*np.sin(Theta2[0]), 0]
        p2 = [self.L1*np.cos(Theta2[1]), self.L1*np.sin(Theta2[1]), 0]
        c = np.cross(p1, p2)
        if (c[2]<0):
            desiredJoint2 = Theta2[0]
        else:
            desiredJoint2 = Theta2[1]
        desiredJoint3 = self.solTheta3(self.L1, self.L2, desiredJoint2, z)
        k = 0
        solution = self.directKinematics(self.L1, self.L2, self.L3,
                                         desiredJoint1, 
                                         desiredJoint2,
                                         desiredJoint3[k]) 
        solutionChecker = np.linalg.norm(solution-desiredPosition)
        if solutionChecker > 0.01:
            k = 1
            solution = self.directKinematics(self.L1, self.L2, self.L3,
                                             desiredJoint1, 
                                             desiredJoint2,
                                             desiredJoint3[k])
            solutionChecker = np.linalg.norm(solution-desiredPosition)
            if solutionChecker > 0.01:
                print("Check for errors in the inverse kinematics")
                return
        self.desiredJoint1 = desiredJoint1*180/np.pi
        self.desiredJoint2 = desiredJoint2*180/np.pi    
        self.desiredJoint3 = desiredJoint3[k]*180/np.pi
        self._desiredPosition = solution
    
    def showInfo(self) -> None:
        self.__getJointPosition()
        print(self.actualPosition)
        
def main():
    robot = DMR_bot()
    # r = 200/2
    robot.initialPosition()
    # for j in range(3):    
    #     for i in range(4):
    #         robot.robotSynchronousPositionControl(180.00+r*np.cos(i*90*np.pi/180), 
    #                                               292.81+r*np.sin(i*90*np.pi/180), 
    #                                               500-(j +1)*20, 150)
    
    # robot.showInfo()
    # robot.teachingMode()
    robot.disconnect()
    
if __name__ == "__main__":
    main()
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        