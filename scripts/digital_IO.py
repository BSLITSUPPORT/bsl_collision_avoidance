#!/usr/bin/env python

#####################################################################
# NODE DETAILS:                                                     #
# This node recives messages from detected_objects topic and        #
# determines if there is an obstruction in the path of the CTA.     #
# If so the digital output pin, determined by the modbus_address    #
# parameter is set LOW and if no obstruction exists the pin is set  #
# HIGH.                                                             #
#                                                                   #
# PARAMETERS:                                                       #
#     - modbus_address:   Defines the modbus address of the digital #
#                            output pin.                            #
#                                                                   #
# TOPICS:                                                           #
#    SUBSCRIBED:                                                    #
#        - detected_objects                                         #
#    PUBLISHED:                                                     #
#        - none                                                     #
#####################################################################

import rospy
from visualization_msgs.msg import MarkerArray
from pyModbusTCP.client import ModbusClient

def callback(marker_array, address):
    #Create a connection with the ADAM-6052 
    adam = ModbusClient(host="192.168.1.3", port=502, auto_open=True, auto_close=False)
    
    #If there is an obstruction detected write pin False
    if len(marker_array.markers) > 0:
        adam.write_single_coil(int(address), False)
    #If there is no obstruction detected write pin True
    else:
        adam.write_single_coil(int(address), True)


if __name__ == '__main__':
    #Initialise node
    rospy.init_node('digital_IO')
    
    #Get Paramaters
    modbus_address = rospy.get_param('~modbus_address')
    
    #Create subscribtion to detected_objetcs topic
    rospy.Subscriber("detected_objects", MarkerArray, callback, modbus_address)
    
    #Hold node open until ROS system is shutdown
    rospy.spin()

    
