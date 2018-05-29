#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from pyModbusTCP.client import ModbusClient

def callback(marker_array, pin):
	adam = ModbusClient(host="192.168.1.3", port=502, auto_open=True, auto_close=False)
	
	if len(marker_array.markers) > 0:
		if adam.write_single_coil(16+int(pin), True):
			print "Digital Write to True Succesful"
		else:
			print "Can't connect to ADAM"
	else:
		if adam.write_single_coil(16+int(pin), False):
			print "Digital Write to False Succesful"
		else:
			print "Can't connect to ADAM"

if __name__ == '__main__':
	rospy.init_node('digital_IO', anonymous=True)
	
	digital_pin = rospy.get_param('~digital_pin')
	rospy.Subscriber("detected_objects", MarkerArray, callback, digital_pin)
	
	rospy.spin()

    
