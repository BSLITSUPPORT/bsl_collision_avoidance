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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from pyModbusTCP.client import ModbusClient

def callback(marker_array, inputs):
    output_address = inputs[0] 
    input_address = inputs[1]
    zone_distance = inputs[2]
    grid_x = inputs[3]
    #Create a connection with the ADAM-6052 
    adam = ModbusClient(host="192.168.1.3", port=502, auto_open=True, auto_close=False)
    
    #Calculate distance to nearest object
    distance = 100
    for marker in marker_array.markers:
        xpos = marker.pose.position.x
        xsize = marker.scale.x
        #Set distance if new distance is smaller
        if xpos - float(xsize)/2 - grid_x < distance: 
            distance = xpos - float(xsize)/2 - grid_x
    
    print distance
    
    a = adam.read_discrete_inputs(int(input_address))
    #If it is a valid read
    if type(a) == type([]):
        #If HIGH
        if a[0] == 1:
            #If there is an obstruction less than xm detected write pin False
            if distance <= int(zone_distance):
                adam.write_single_coil(int(output_address), False)
            #If there is an no obstruction less than xm detected write pin False
            else:
                adam.write_single_coil(int(output_address), True)
        #If LOW
        elif a[0] == 0:
            #If there is an obstruction detected write pin False
            if len(marker_array.markers) > 0:
                adam.write_single_coil(int(output_address), False)
            #If there is no obstruction detected write pin True
            else:
                adam.write_single_coil(int(output_address), True)

if __name__ == '__main__':
    #Initialise node
    rospy.init_node('digital_IO')
    
    #Get Paramaters
    zone_distance = rospy.get_param('~zone_distance')
    grid_x = rospy.get_param('~grid_x')
    modbus_address_output = rospy.get_param('~modbus_address_output')
    modbus_address_input = rospy.get_param('~modbus_address_input')
    
    #Create subscribtion to detected_objetcs topic
    rospy.Subscriber("detected_objects", MarkerArray, callback, [modbus_address_output, modbus_address_input, zone_distance, grid_x])
    
    #Hold node open until ROS system is shutdown
    rospy.spin()

    
