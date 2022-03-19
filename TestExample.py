# -*- coding: utf-8 -*-
"""
Created on Wed Mar  9 18:17:53 2022

@author: jsanjuan
"""

import DMR_bot as robotLibrary

robot = robotLibrary.DMR_bot()
robot.enableRobotSynchronousPositionControl()
robot.robotSynchronousPositionControl(180, 292.81, 500, 100)
robot.robotSynchronousPositionControl(180, 292.81, 500, 100)
robot.robotSynchronousPositionControl(x, y, z, linearVel)
robot.disconnect()