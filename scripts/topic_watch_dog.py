#!/usr/bin/env python

#####################################################################
# NODE DETAILS:														#
# This node monitors a topic and determines if it is recieving		#
# data. If no data is recieved from the topic in the time defined	#
# by the timeout parameter, it publishes that the topic has failed.	#
#																	#
# PARAMETERS:														#
# 	- topic: 		Defines which topic to Monitor.					#
#	- topic_type: 	Defines the message type of the monitored topic.#
#	- timeout:		Defines the period of time where no messages  	#
#						are recieved before a failure is noticed.	#
#																	#
# TOPICS:															#
#	SUBSCRIBED:														#
#		- 							#
#	PUBLISHED:														#
#		- 							#
#####################################################################

import rospy
import std_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
from time import sleep
from bsl_collision_avoidance.msg import TopicFailure

class SubscribeAndPublish(object):
    def __init__(self, topic, topic_type, timeout):
        #Initate OccupancyGrid Publisher
        self.pub = rospy.Publisher('failures', TopicFailure, queue_size=1)
        
        #Initiate Point Cloud Subscriber
        self.sub = rospy.Subscriber(topic, eval(topic_type), self.callback)
        
        self.triggered = rospy.get_time()
        while not rospy.is_shutdown():
			if rospy.get_time() - self.triggered >= timeout:
				failureMsg = TopicFailure()
				failureMsg.topic_name = rospy.get_namespace()+topic
				failureMsg.is_alive = 0
				self.pub.publish(failureMsg)
			else:
				successMsg = TopicFailure()
				successMsg.topic_name = rospy.get_namespace()+topic
				successMsg.is_alive = 1
				self.pub.publish(successMsg)
			sleep(timeout/3)
			
    
    def callback(self, cloud):
		self.triggered = rospy.get_time()
		

if __name__ == '__main__':
	#Initialise node
	rospy.init_node('topic_watch_dog')
	
	#Get parameters
	topic = rospy.get_param('~topic')
	topic_type = rospy.get_param('~topic_type')
	timeout = rospy.get_param('~timeout')
	
	#Subscribe
	a = SubscribeAndPublish(topic, topic_type, timeout)
	
	rospy.spin()
	
	
	
