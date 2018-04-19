#!/usr/bin/env /usr/bin/python
import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from time import sleep
from math import pi
    
def laser_staticbroadcaster(laser, x, y, z):
    br = tf2_ros.StaticTransformBroadcaster()
    
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

def degTOrad(deg):
    return float(deg)*pi/180
    
if __name__ == '__main__':
    rospy.init_node('tf2_laser_angle_broadcaster')
    laser_staticbroadcaster(1,0,0,0)
    sleep(1)
    laser_staticbroadcaster(2,0,0,0)
    
    print "You have enter the LiDAR Calibration tool"
    print "Adjusting LIDAR 1"
    laser = 1
    while 1:
        degrees = raw_input("What is your desired angle? (x, y, z) 'q' to quit. ").split(', ')
        if len(degrees) == 3:
            radians = map(degTOrad, degrees)
            laser_staticbroadcaster(laser, radians[0], radians[1], radians[2])
        elif degrees[0] == 'q':
            if laser == 1:
                print "Adjusting LIDAR 2"
                laser = 2
            elif laser == 2:
                break
        sleep(1)
    print "You have stopped making changes"
    print "You have quit the LiDAR Claibration Tools"
    rospy.spin()
