#!/usr/bin/env /usr/bin/python
import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from time import sleep
from math import pi
import rospkg

    
def laser_staticbroadcaster(laser, x, y, z):
    br = tf2_ros.StaticTransformBroadcaster()
    sleep(1)
    
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    if laser == 1:
        t.header.frame_id = "laser_mount"
        t.child_frame_id = "laser"
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

def degTOrad(deg):
    return float(deg)*pi/180
    
if __name__ == '__main__':
    #Initiate Node
    rospy.init_node('tf2_laser_angle_broadcaster')
    
    #read existing laser angles from file
    rospack = rospkg.RosPack()
    path = rospack.get_path('bsl_pkg')
    f = open(path+'/include/systemProperties.txt', 'r')
    laser1angle = map(float, f.readline().split(','))
    laser2angle = map(float, f.readline().split(','))
    f.close()
    
    print laser1angle
    
    #Load first angle
    laser_staticbroadcaster(1,0,0,0)
    laser_staticbroadcaster(2,laser2angle[0],laser2angle[1],laser2angle[2])
    laser_staticbroadcaster(1,laser1angle[0],laser1angle[1],laser1angle[2])
    
    rospy.spin()
