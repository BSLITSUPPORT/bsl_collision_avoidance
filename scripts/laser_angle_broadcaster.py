#!/usr/bin/env python
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
import rospkg


def laser_broadcaster(laser_frame, parent_frame, x, y, z):
    br = tf2_ros.StaticTransformBroadcaster()
    
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = laser_frame
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0.1
    
    q = tf.transformations.quaternion_from_euler(x, y, z)
    
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    br.sendTransform(t)
    
if __name__ == '__main__':
	#Initiate Node
	rospy.init_node('laser_angle_broadcaster')

	#Get ROS Paramaters
	laser_frame = rospy.get_param('~laser_frame')
	parent_frame = rospy.get_param('~parent_frame')

	#Locate file
	rospack = rospkg.RosPack()
	path = rospack.get_path('bsl_collision_avoidance')

	#Read angles from file
	f = open(path+'/config/'+laser_frame+'.txt', 'r')
	laserangle = map(float, f.readline().split(','))
	f.close()

	#Broadcast Laser Angles
	laser_broadcaster(laser_frame, parent_frame, laserangle[0], laserangle[1], laserangle[2])

	rospy.spin()
