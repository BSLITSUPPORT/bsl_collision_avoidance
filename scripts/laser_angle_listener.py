#!/usr/bin/env python
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
import rospkg

def callback(t):
	br = tf2_ros.StaticTransformBroadcaster()
	br.sendTransform(t)

	laser_frame = t.child_frame_id

	rospack = rospkg.RosPack()
	path = rospack.get_path('bsl_collision_avoidance')
	f = open(path+'/config/'+laser_frame+'.txt', 'w')
	angles = tf.transformations.euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
	angles = ", ".join(map(str,angles))
	f.write(angles)
	f.close()

	print angles
	
	
    
if __name__ == '__main__':
	#Initiate Node
	rospy.init_node('laser_angle_listener')

	sub = rospy.Subscriber('calibratorTransform', TransformStamped, callback)

	rospy.spin()
