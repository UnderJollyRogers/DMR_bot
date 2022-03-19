# -*- coding: utf-8 -*-
"""
Motor class(self, nodeNumber, incPerRev:int, stallTorque(Nm))
-node
-Position
-Velocity
---------------------
-getPosition
-motorEnable
-posControl

If the speed of the program is too slow, consider using PDO instead of SDO.

@author: jsanjuan
"""
import canopen
import time
# import math
import numpy
from typing import TypeVar

function = TypeVar('Function')
class Motor:
    network = canopen.Network()
    network.connect(bustype='ixxat', channel=0, bitrate=1000000)
    def __init__(self, nodeNumber: int, incPerRev: int, gearRatio: int, nominalTorque:float) -> None:
        self.node = Motor.network.add_node(nodeNumber, 'od.eds') 
        self._incPerRev = incPerRev
        self._nodeNumber = nodeNumber
        self._gearRatio = gearRatio
        self._nominalTorque = nominalTorque #Torque constant*Nominal current
        self._dictionaryOfOperations = { 0:"idleState",
                                        1:"positionControl",
                                        8:"synchronousPositionControl",
                                       10:"torqueControl"}
        self._modeOfOperation = 0
        
    
    """ Properties Getter """
    @property
    def position(self) -> int:
        return self.node.sdo["Position actual value"].raw*360/self.incPerRev
    
    @property
    def incPerRev(self):
        return self._incPerRev 
    
    @property        
    def nodeNumber(self):
        return self._nodeNumber
    
    @property
    def gearRatio(self):
        return self._gearRatio
        
    @property
    def modeOfOperation(self):
        return self._dictionaryOfOperations[self._modeOfOperation]
    
    @property
    def nominalTorque(self):
        return self._nominalTorque
    
    """ Properties Setter """
    @modeOfOperation.setter
    def modeOfOperation(self, mode:int) -> None:
        self._modeOfOperation = mode
    
    """ Decoration Shelf """
    def _motorEnableDecorator(instruction:function) -> function:
        def stateSwitching(self):
            instruction(self)
            if self.modeOfOperation == "synchronousPositionControl":
                self.node.nmt.state = 'OPERATIONAL'
                self.node.rpdo[2]['Controlword'].raw = 0x0000 
                self.node.rpdo[2]['Modes of operation'].raw = 8 
                self.node.rpdo[2].transmit()
                time.sleep(0.5)
                self.node.rpdo[2]['Controlword'].raw = 0x0006 
                self.node.rpdo[2]['Modes of operation'].raw = 8 
                self.node.rpdo[2].transmit()
                time.sleep(0.5)
                self.node.rpdo[2]['Controlword'].raw = 0x0007 
                self.node.rpdo[2]['Modes of operation'].raw = 8 
                self.node.rpdo[2].transmit()
                time.sleep(0.5)
                self.node.rpdo[2]['Controlword'].raw = 0x000F 
                self.node.rpdo[2]['Modes of operation'].raw = 8 
                self.node.rpdo[2].transmit()
                time.sleep(0.5)
            else:
                self.node.nmt.state = "PRE-OPERATIONAL"
                self.node.sdo["Controlword"].raw = 0x0000 #Switch on disabled
                time.sleep(0.5)
                if self.modeOfOperation != "idleState" : 
                    self.node.sdo["Controlword"].raw = 0x0006 #Ready to switch on
                    time.sleep(0.5)
                    self.node.sdo["Controlword"].raw = 0x0007 #Switched on
                    time.sleep(0.5)
                    if self.modeOfOperation == "torqueControl" :
                        self.node.sdo['Controlword'].raw=0x000F
                        time.sleep(0.5)
        return stateSwitching
        
    def _verifyConfigurationDecorator(instruction:function) -> None:
        def checkConfiguration(self, *args):
            if instruction.__name__ == self.modeOfOperation:
                instruction(self, *args)
            else:
                print("The motor node "+str(self._nodeNumber)+" is not enabled or is in the wrong operation mode.")
        return checkConfiguration
    """ public Methods """
    
    @_motorEnableDecorator
    def enablePositionControl(self) -> None:
        self.node.sdo["Modes of operation"].raw = 1
        self.modeOfOperation = 1
        
    @_motorEnableDecorator
    def enableTorqueControl(self) -> None:
        self.node.sdo["Modes of operation"].raw = 10
        self.modeOfOperation = 10
    
    @_motorEnableDecorator
    def enableSynchronousPositionControl(self) -> None:
        self.modeOfOperation = 8
        self.__tpdoConfiguration()
        self.__rpdoConfiguration()
    
    @_verifyConfigurationDecorator
    def positionControl(self, targetPosition:int, profileVelocity:float) -> None:
        if profileVelocity < 10:
            profileVelocity = 10
        rpm = profileVelocity*self.gearRatio/6
        self.node.sdo["Controlword"].raw = 0x000F
        self.node.sdo["Target position"].raw = round(targetPosition*self.incPerRev/360)
        self.node.sdo["Profile velocity"].raw = round(rpm)
        self.node.sdo["Controlword"].raw = 0x003F #Refresh
    
    def synchronousPositionControl(self, targetPosition:int, offsetTorque:int) -> None:
        self.node.rpdo[1]['Controlword'].raw=0x000F 
        self.node.rpdo[1]['Target position'].raw = round(targetPosition*self.incPerRev/360)
        self.node.rpdo[1]['Torque offset'].raw =  offsetTorque*10/self.nominalTorque
        self.node.rpdo[1].transmit()
    
    @_verifyConfigurationDecorator
    def torqueControl(self, targetTorque: float):
        self.node.sdo['Target torque'].raw = targetTorque*10/self.nominalTorque
        # if self.nodeNumber == 2:
            # print(targetTorque*10/self.nominalTorque)
        self.node.sdo['Controlword'].raw=0x000F
        
    @_motorEnableDecorator
    def disableMotor(self):
        self.modeOfOperation = 0
    
    """ private Methods """
    def __tpdoConfiguration(self) -> None:
        self.node.tpdo[1].read()
        self.node.tpdo[2].read()
        
    def __rpdoConfiguration(self) -> None:
        self.node.rpdo[1].read()
        self.node.rpdo[2].read()
        
    @staticmethod
    def disconnect() -> None:
        Motor.preOperationalMode()
        Motor.network.disconnect()
    
    @staticmethod
    def preOperationalMode() -> None:
        Motor.network.nmt.state = 'PRE-OPERATIONAL'
        
def main():
    # motor1 = Motor(1, 4800, 100, 1.146)
    # motor1.enableSynchronousPositionControl()
    # sendingPeriod = 0.001
    # print(motor1.position)
    # A = numpy.linspace(0, 90, num=1000)
    # for i in range(len(A)):
    #     motor1.synchronousPositionControl(A[i], 0)
    #     initialTime = time.time()
    #     while True:
    #         finalTime = time.time()
    #         if finalTime - initialTime > sendingPeriod:
    #             print(motor1.position)
    #             break
    # print(motor1.position)
    # motor1.enableTorqueControl()
    # motor1.torqueControl(30)
    # time.sleep(2)
    # motor1.torqueControl(-30)
    # time.sleep(2)
    # motor1.torqueControl(0.0)
    # motor1.enablePositionControl()
    # motor1.positionControl(0, 100)
    # time.sleep(5)
    
    
    
    
    motor2 = Motor(2, 4800, 100, 1.46)
    # motor3 = Motor(3, 7200, 100, 1.146)
    
    motor2.enableTorqueControl()
    motor2.torqueControl(-10)
    
    # motor3.enablePositionControl()
    
    # time.sleep(2)
    # motor1.positionControl(90, 100)
    # motor2.positionControl(0, 100)
    # motor3.positionControl(-65, 10)
    # time.sleep(5)
    # motor3.positionControl(0, 10)
    # print(motor1.getPosition())
    # print(motor2.getPosition())
    # print(motor3.getPosition())
    # print(motor3.position)
    Motor.disconnect()

if __name__ == "__main__":
    main()
        
        