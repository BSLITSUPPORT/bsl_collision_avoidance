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
    rospy.init_node('calibration_talker')
    
    #Initialise Publisher to calibrator topic
    pub = rospy.Publisher('calibrator', TransformStamped, queue_size=1)
    
    #Calibration section for left LIDAR
    print "You have entered the LiDAR Calibration tool"
    print "\nAdjusting LIDAR 1"
    laser = 1
    laser_frame = rospy.get_param('/1/initialise_LIDAR_transform/laser_frame')
    parent_frame = rospy.get_param('/1/initialise_LIDAR_transform/parent_frame')
    x = float(rospy.get_param('/1/x'))
    y = float(rospy.get_param('/1/y'))
    z = float(rospy.get_param('/1/z'))
    current_height = float(rospy.get_param('/1/height'))
    current_angle_rad = [x, y, z]
    current_angle_deg = map(radTodeg, current_angle_rad)
    transform1 = get_transform(laser_frame, parent_frame, x, y, z, current_height)
    pub.publish(transform1)
    while True:
        #Asks user for rotation for LIDAR transform
        print "The current angle of "+laser_frame+" is: "+str(current_angle_deg)
        print "The current height of "+laser_frame+" is: "+str(3.8 - current_height)
        degrees = raw_input("What is your desired angle? (x, y, z, height) 'q' to quit. ").split(', ')
        #If user inputs rotation matrix
        if len(degrees) == 4:
            current_angle_deg = degrees[0:3]
            current_height = 3.8 - float(degrees[3])
            current_angle_rad = map(degTOrad, current_angle_deg)
            #Publish transform determined from user input
            transform1 = get_transform(laser_frame, parent_frame, current_angle_rad[0], current_angle_rad[1], current_angle_rad[2], current_height)
            pub.publish(transform1)
        #If user wants to quit calibration
        elif degrees[0] == 'q':
            #quit
            break
        #If user input does not meet requirements
        else:
            print "You have not given a valid Input"
        sleep(1)
    
    
    print "Quitting "+laser_frame+" claibration. Saving given parameters. This may take some time."    
    rospy.set_param('/1/x', str(current_angle_rad[0]))
    rospy.set_param('/1/y', str(current_angle_rad[1]))
    rospy.set_param('/1/z', str(current_angle_rad[2]))
    rospy.set_param('/1/height', str(current_height))
        
    print "\nAdjusting LIDAR 2"
    laser = 2
    laser_frame = rospy.get_param('/2/initialise_LIDAR_transform/laser_frame')
    parent_frame = rospy.get_param('/2/initialise_LIDAR_transform/parent_frame')
    x = float(rospy.get_param('/2/x'))
    y = float(rospy.get_param('/2/y'))
    z = float(rospy.get_param('/2/z'))
    current_height = float(rospy.get_param('/2/height'))
    current_angle_rad = [x, y, z]
    current_angle_deg = map(radTodeg, current_angle_rad)
    transform2 = get_transform(laser_frame, parent_frame, x, y, z, current_height)
    pub.publish(transform2)
    while True:
        #Asks user for rotation for LIDAR transform
        print "The current angle of "+laser_frame+" is: "+str(current_angle_deg)
        print "The current height of "+laser_frame+" is: "+str(3.8 - current_height)
        degrees = raw_input("What is your desired angle? (x, y, z, height) 'q' to quit. ").split(', ')
        #If user inputs rotation matrix
        if len(degrees) == 4:
            current_angle_deg = degrees[0:3]
            current_height = 3.8 - float(degrees[3])
            current_angle_rad = map(degTOrad, current_angle_deg)
            #Publish transform determined from user input
            transform2 = get_transform(laser_frame, parent_frame, current_angle_rad[0], current_angle_rad[1], current_angle_rad[2], current_height)
            pub.publish(transform2)
        #If user wants to quit calibration
        elif degrees[0] == 'q':
            #quit
            break
        #If user input does not meet requirements
        else:
            print "You have not given a valid Input"
        sleep(1)
    
    print "Quitting "+laser_frame+" claibration. Saving given parameters. This may take some time."    
    rospy.set_param('/2/x', str(current_angle_rad[0]))
    rospy.set_param('/2/y', str(current_angle_rad[1]))
    rospy.set_param('/2/z', str(current_angle_rad[2]))
    rospy.set_param('/2/height', str(current_height))

    print "You have stopped making changes your changes have been saved"
    print "Quiting the LiDAR Claibration Tool"
    print "Make sure you power cycle the machine to make changes permanent"
    sleep(2)
