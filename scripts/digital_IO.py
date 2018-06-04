#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray
from pyModbusTCP.client import ModbusClient

def callback(marker_array, pin):
    #Create a connection with the ADAM-6052 
	adam = ModbusClient(host="192.168.1.3", port=502, auto_open=True, auto_close=False)
	
	#If there is an obstruction detected write pin False
	if len(marker_array.markers) > 0:
		adam.write_single_coil(16+int(pin), False)
	#If there is no obstruction detected write pin True
	else:
		adam.write_single_coil(16+int(pin), True)


if __name__ == '__main__':
	#Initialise node
	rospy.init_node('digital_IO')
	
	#Get Paramaters
	digital_pin = rospy.get_param('~digital_pin')
	
	#Create subscribtion to detected_objetcs topic
	rospy.Subscriber("detected_objects", MarkerArray, callback, digital_pin)
	
	#Hold node open until ROS system is shutdown
	rospy.spin()

    
