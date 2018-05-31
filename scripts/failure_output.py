#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from time import sleep
from pyModbusTCP.client import ModbusClient

class subscriber:
	def __init__(self):
		self.failureTimes = {}
		self.failureValue = {}
		faultTime = rospy.get_time()
		adam = ModbusClient(host="192.168.1.3", port=502, auto_open=True, auto_close=False)
		
		for topic_pair in rospy.get_published_topics():
			topic = topic_pair[0]
			if topic[3:] == "failures":
				rospy.Subscriber(topic, String, self.callback)
				
		while not rospy.is_shutdown():
			for key in self.failureTimes.keys():
				if rospy.get_time() - self.failureTimes[key] >= 2:
					adam.write_single_coil(16, False)
					print "fault"
					faultTime = rospy.get_time()
			if rospy.get_time() - faultTime >= 2:
				adam.write_single_coil(16, True)
				print "no fault"
					
			sleep(0.1)
		
	def callback(self, msg):
		topic, value = msg.data.split(" ")
		
		if int(value) == 1:
			self.failureTimes[topic] = rospy.get_time()
			self.failureValue[topic] = int(value)

	
if __name__ == '__main__':
	rospy.init_node('failure_output')
	
	sleep(6)

	a = subscriber()
	
	rospy.spin()
