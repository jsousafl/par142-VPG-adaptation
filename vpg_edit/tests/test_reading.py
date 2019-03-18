#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import urx

# I'M USING A FALSE ROBOT IN HERE DEFINED JUST FOR THE URX LIBRARY IN ORDER TO MAKE IT EASIER 
# It means : no need for defining all features of Robot class

false_robot = urx.Robot("192.168.1.5")
actual_tool_pose = false_robot.getl(wait=True)
false_robot.close()
print("Actual tool pose : {}".format(actual_tool_pose))