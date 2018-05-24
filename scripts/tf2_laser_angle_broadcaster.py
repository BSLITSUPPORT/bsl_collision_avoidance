#!/usr/bin/env python
import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from time import sleep
import rospkg

    
def laser_staticbroadcaster(laser, x, y, z):
    br = tf2_ros.StaticTransformBroadcaster()
    
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    if laser == 1:
        t.header.frame_id = "laser_mount"
        t.child_frame_id = "laser1"
    elif laser == 2:
        t.header.frame_id = "laser_mount2"
        t.child_frame_id = "laser2"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0.1
    
    q = tf.transformations.quaternion_from_euler(x, y, z)
    
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    br.sendTransform(t)
    sleep(1)
    
if __name__ == '__main__':
    #Initiate Node
    rospy.init_node('tf2_laser_angle_broadcaster')
    rate = rospy.Rate(10)
    
    #Open file
    rospack = rospkg.RosPack()
    path = rospack.get_path('bsl_collision_avoidance')
    f = open(path+'/include/systemProperties.txt', 'r')
    
    #Read first line which contains laser1 angles
    laser1angle = map(float, f.readline().split(','))
    laser2angle = map(float, f.readline().split(','))

    f.close()
    
    #Broadcast Laser1 Angles
    laser_staticbroadcaster(1,laser1angle[0],laser1angle[1],laser1angle[2])
    laser_staticbroadcaster(2,laser2angle[0],laser2angle[1],laser2angle[2])
    
    rospy.spin()
