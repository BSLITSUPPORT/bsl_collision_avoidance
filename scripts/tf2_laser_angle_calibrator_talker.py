#! /usr/bin/env python
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

# Converts a number from degrees to radians
def degTOrad(deg):
    return float(deg)*pi/180

# Converts a number from radians to degrees
def radTodeg(rad):
    return float(rad)*180/pi
    
if __name__ == '__main__':
    #Initiate Node
    rospy.init_node('tf2_laser_angle_calibrator')
    
    #read existing laser angles from file
    rospack = rospkg.RosPack()
    path = rospack.get_path('bsl_collision_avoidance')
    f = open(path+'/include/systemProperties.txt', 'r')
    laser1angle = map(float, f.readline().split(','))
    laser2angle = map(float, f.readline().split(','))
    f.close()
    
    #Load first angle
    laser_staticbroadcaster(1,0,0,0)
    laser_staticbroadcaster(1,laser1angle[0],laser1angle[1],laser1angle[2])
    laser_staticbroadcaster(2,laser2angle[0],laser2angle[1],laser2angle[2])
    
    #Calibration section user prompts
    print "You have entered the LiDAR Calibration tool"
    print "\nAdjusting LIDAR 1"
    laser = 1
    current_angle_rad = [laser1angle[0],laser1angle[1],laser1angle[2]]
    current_angle_deg = map(radTodeg, current_angle_rad)
    radians = current_angle_rad
    while 1:
        print "The current angle of LiDAR "+str(laser)+" is: "+str(current_angle_deg)
        degrees = raw_input("What is your desired angle? (x, y, z) 'q' to quit. ").split(', ')
        if len(degrees) == 3:
            current_angle_deg = degrees
            radians = map(degTOrad, degrees)

            laser_staticbroadcaster(laser, radians[0], radians[1], radians[2])
        elif degrees[0] == 'q':
            if laser == 1:
                final_angle_rad_1 = radians
                print "\nAdjusting LIDAR 2"
                current_angle_rad = [laser2angle[0],laser2angle[1],laser2angle[2]]
                current_angle_deg = map(radTodeg, current_angle_rad)
                radians = current_angle_rad
                laser = 2
            elif laser == 2:
                final_angle_rad_2 = radians
                break
        sleep(1)
    print "You have stopped making changes"
    while 1:
        answer = raw_input("Would you like to commit these changes? 'y'/'n' ").lower()
        if len(answer) == 1:
            if answer == 'y':
                #write to file
                f = open(path+'/include/systemProperties.txt', 'w')
                f.write(','.join(map(str, final_angle_rad_1))+'\n')
                f.write(','.join(map(str, final_angle_rad_2))+'\n')
                f.close()
                break
            elif answer == 'n':
                break
        print "Make sure you just enter a single letter"
    print "You have quit the LiDAR Claibration Tools"
