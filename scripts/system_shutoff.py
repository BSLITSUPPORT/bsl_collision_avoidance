#!/usr/bin/env python

#####################################################################
# Node Details:														#
# 																	#
# This Node checks to see if the digital pin 0 is high. If so 		#
# it turns the off the computer.									#
#####################################################################

import rospy
from pyModbusTCP.client import ModbusClient
import os
from time import sleep

if __name__ == '__main__':
    #Initiate Node
	rospy.init_node('system_shutoff')
	
	adam = ModbusClient(host="192.168.1.3", port=502, auto_open=True, auto_close=False)
	
	#While 
	while not rospy.is_shutdown():
		a = adam.read_discrete_inputs(0)
		if type(a) == type([]):
			if a[0] == True:
				os.system('systemctl poweroff')
		sleep(0.1)
				
	#Hold node open until ROS system is shutdown
	rospy.spin()

