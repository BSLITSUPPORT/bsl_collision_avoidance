#!/usr/bin/env python

#####################################################################
# Node Details:														#
# 																	#
# This node listens to the calibrator topic.						#
# When a message is recieved it adjusts and saves 					#
# the LIDAR transform.												#
#####################################################################

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
import rospkg

def applyTransform(t):
    #Broadcast recieved transform
	br = tf2_ros.StaticTransformBroadcaster()
	br.sendTransform(t)
	
	#Write transform to file
	laser_frame = t.child_frame_id
	rospack = rospkg.RosPack()
	path = rospack.get_path('bsl_collision_avoidance')
	f = open(path+'/config/'+laser_frame+'.txt', 'w')
	angles = tf.transformations.euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
	angles = ", ".join(map(str,angles))
	f.write(angles)
	f.close()
    
if __name__ == '__main__':
	#Initiate Node
	rospy.init_node('calibration_listener')
	
	#Create a subscribtion to the 
	rospy.Subscriber('calibrator', TransformStamped, applyTransform)
	
	#Hold node open until ROS system is shutdown
	rospy.spin()
