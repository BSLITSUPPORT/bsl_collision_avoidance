#!/usr/bin/env python

#####################################################################
# NODE DETAILS:                                                     #
# This Node checks to see if the digital pin 0 is HIGH. If so       #
# it turns the off the computer.                                    #
#                                                                   #
# PARAMETERS:                                                       #
#     - none                                                        #
#                                                                   #
# TOPICS:                                                           #
#    SUBSCRIBED:                                                    #
#        - none                                                     #
#    PUBLISHED:                                                     #
#        - none                                                     #
#####################################################################

import rospy
from pyModbusTCP.client import ModbusClient
import os
from rospy import sleep

if __name__ == '__main__':
    #Initiate Node
    rospy.init_node('system_shutoff')
    
    #Get ROS parameters
    modbus_address = rospy.get_param('~modbus_address')
    
    #Initialise communication with ModbusClient
    adam = ModbusClient(host="192.168.1.3", port=502, auto_open=True, auto_close=False)
    
    #While system is not shutdown
    while not rospy.is_shutdown():
        #Read shutdown pin
        a = adam.read_discrete_inputs(int(modbus_address))
        #If it is a valid read
        if type(a) == type([]):
            #If pin is HIGH
            if a[0] == True:
                #Shutdown System
                os.system('systemctl poweroff')
        sleep(0.1)
                
    #Hold node open until ROS system is shutdown
    rospy.spin()

