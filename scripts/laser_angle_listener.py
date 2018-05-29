#!/usr/bin/env python
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
import rospkg

def callback(transform):
    br = tf2_ros.StaticTransformBroadcaster()
    br.sendTransform(transform)
    
    #and write to file
    
if __name__ == '__main__':
	#Initiate Node
	rospy.init_node('laser_angle_listener')

	sub = rospy.Subscriber('calibratorTransform', TransformStamped, callback)

	rospy.spin()
