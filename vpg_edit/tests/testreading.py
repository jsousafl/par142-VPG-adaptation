#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 27 15:40:28 2018

@author: jsousafl
"""

import urx
import numpy as np
import math

#I'M USING A FALSE ROBOT IN HERE DEFINED JUST FOR THE URX LIBRARY IN ORDER TO MAKE IT EASIER 
# this means : no need for defining all features of Robot class

false_rob = urx.Robot("192.168.1.5")
actual_tool_pose = false_rob.getl(wait=True)
false_rob.close()
print(actual_tool_pose)
#tool_position = [0.2942285170783312, 0.29410337438792217, 0]
#tool_orientation = [-2.999474361006964, 0.8146729921637934, -0.02363364903072137]
#robot.move_to(tool_position, tool_orientation)