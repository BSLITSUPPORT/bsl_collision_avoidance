#!/usr/bin/env python

#####################################################################
# Node Details:														#
# 																	#
# This node recieves all the messages from every topic watchdog		#
# and determines if a failure has occured. If so it sets the fault  #
# pin to LOW and doesn't switch it back until 2 seconds after the	#
# system has been restored.											#
#####################################################################

import rospy
from time import sleep
from pyModbusTCP.client import ModbusClient
from bsl_collision_avoidance.msg import TopicFailure

class Subscriber:
	def __init__(self):
		self.failureTimes = {}
		self.failureValue = {}
		faultTime = rospy.get_time()
		adam = ModbusClient(host="192.168.1.3", port=502, auto_open=True, auto_close=False)
		
		for topic_pair in rospy.get_published_topics():
			topic = topic_pair[0]
			if topic[3:] == "failures":
				rospy.Subscriber(topic, TopicFailure, self.callback)
				
		while not rospy.is_shutdown():
			if len(self.failureTimes.keys()) == 0:
				adam.write_single_coil(16, False)
			else:
				for key in self.failureTimes.keys():
					if rospy.get_time() - self.failureTimes[key] >= 2:
						adam.write_single_coil(16, False)
						print "fault"
						faultTime = rospy.get_time()
				if rospy.get_time() - faultTime >= 2:
					adam.write_single_coil(16, True)
					print "no fault"						
		
	def callback(self, msg):
		topic = msg.topic_name
		value = msg.is_alive
		
		if value == 1:
			self.failureTimes[topic] = rospy.get_time()
			self.failureValue[topic] = value

	
if __name__ == '__main__':
	rospy.init_node('failure_output')
	
	sleep(6)

	a = Subscriber()
	
	rospy.spin()
