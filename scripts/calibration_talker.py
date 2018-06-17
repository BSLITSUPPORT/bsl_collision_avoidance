#! /usr/bin/env python

#####################################################################
# NODE DETAILS:                                                     #
# This Node is used to calibrate the LIDAR transforms, it does      #
# this by requesting a user input and broadcasting that transform   #
# to the calibrator topic.                                          #
#                                                                   #
# PARAMETERS:                                                       #
#     - none                                                        #
#                                                                   #
# TOPICS:                                                           #
#    SUBSCRIBED:                                                    #
#        - none                                                     #
#    PUBLISHED:                                                     #
#        - calibrator                                               #
#####################################################################

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from time import sleep
from math import pi
import rospkg

def get_transform(child_frame, parent_frame, x, y, z, height):
    #Create Transform with specifications given from inputs
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = height
    q = tf.transformations.quaternion_from_euler(x, y, z)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    #Return Transform
    return t

def degTOrad(deg):
    #Convert degrees to radians
    return float(deg)*pi/180

def radTodeg(rad):
    #Convert radians to degrees
    return float(rad)*180/pi
    
if __name__ == '__main__':
    #Initiate Node
    rospy.init_node('tf2_laser_angle_calibrator')
    
    #Initialise Publisher to calibrator topic
    pub = rospy.Publisher('calibrator', TransformStamped, queue_size=1)
    
    #Calibration section user prompts
    print "You have entered the LiDAR Calibration tool"
    print "\nAdjusting LIDAR 1"
    laser = 1
    laser_frame = "laser1"
    parent_frame = "left_lidar_mount"
    current_angle_deg = [0, 0, 0]
    while True:
        #Asks user for rotation for LIDAR transform
        print "The current angle of "+laser_frame+" is: "+str(current_angle_deg)
        degrees = raw_input("What is your desired angle? (x, y, z, height) 'q' to quit. ").split(', ')
        #If user inputs rotation matrix
        if len(degrees) == 4:
            current_angle_deg = degrees[0:3]
            height = 3.8 - current_angle_deg[3]
            radians = map(degTOrad, degrees)
            #Publish transform determined from user input
            pub.publish( get_transform(laser_frame, parent_frame, radians[0], radians[1], radians[2], height) )
        #If user wants to quit calibration
        elif degrees[0] == 'q':
            #If it is the first quit move to second LIDAR
            if laser == 1:
                print "\nAdjusting LIDAR 2"
                laser = 2
                laser_frame = "laser2"
                parent_frame = "right_lidar_mount"
                current_angle_deg = [0, 0, 0]
            #If quitting LIDAR 2 
            elif laser == 2:
                break
        #If user input does not meet requirements
        else:
            print "You have not given a valid Input"
        sleep(1)
    print "You have stopped making changes your changes have been saved"
    print "Quiting the LiDAR Claibration Tool"
    sleep(2)
