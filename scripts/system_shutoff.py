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


    #While system is not shutdown
    while not rospy.is_shutdown():
        #Ping Router
        response = os.system("ping -c 1 192.168.1.12")

        #If router is dead
        if response != 0:
			rospy.loginfo("Connection to the MikroTik has been lost system is shutting down.")
            #Shutdown computer
            os.system('systemctl poweroff')
            
        #Test every second
        sleep(1)
                
    #Hold node open until ROS system is shutdown
    rospy.spin()

