#!/usr/bin/env python

import rospy
import std_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
from time import sleep

class SubscribeAndPublish(object):
    def __init__(self, topic, topic_type, timeout):
        #Initate OccupancyGrid Publisher
        self.pub = rospy.Publisher('failures', std_msgs.msg.String, queue_size=10)
        
        #Initiate Point Cloud Subscriber
        self.sub = rospy.Subscriber(topic, eval(topic_type), self.callback)
        
        self.triggered = rospy.get_time()
        while not rospy.is_shutdown():
			if rospy.get_time() - self.triggered >= timeout:
				self.pub.publish(rospy.get_namespace()+topic+" 0")
			else:
				self.pub.publish(rospy.get_namespace()+topic+" 1")
			sleep(0.1)
			
    
    def callback(self, cloud):
		self.triggered = rospy.get_time()
		

if __name__ == '__main__':
	rospy.init_node('topic_watch_dog')
	
	topic = rospy.get_param('~topic')
	topic_type = rospy.get_param('~topic_type')
	timeout = rospy.get_param('~timeout')
	
	a = SubscribeAndPublish(topic, topic_type, timeout)
	
	rospy.spin()
	
	
	
