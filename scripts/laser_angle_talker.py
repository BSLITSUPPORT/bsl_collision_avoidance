#! /usr/bin/env python
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from time import sleep
from math import pi
import rospkg

def get_transform(laser_frame, parent_frame, x, y, z):
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
    
	return t

# Converts a number from degrees to radians
def degTOrad(deg):
    return float(deg)*pi/180

# Converts a number from radians to degrees
def radTodeg(rad):
    return float(rad)*180/pi
    
if __name__ == '__main__':
    #Initiate Node
    rospy.init_node('tf2_laser_angle_calibrator')
    
    pub = rospy.Publisher('calibratorTransform', Transform, queue_size=1)
    
    #Calibration section user prompts
    print "You have entered the LiDAR Calibration tool"
    print "\nAdjusting LIDAR 1"
    laser = 1
    laser_frame = "laser1"
    parent_frame = "laser_mount1"
    current_angle_deg = [0, 0, 0]
    while True:
        print "The current angle of "+laser_frame+" is: "+str(current_angle_deg)
        degrees = raw_input("What is your desired angle? (x, y, z) 'q' to quit. ").split(', ')
        if len(degrees) == 3:
            current_angle_deg = degrees
            radians = map(degTOrad, degrees)
            pub.publish( get_transform(laser_frame, parent_frame, radians[0], radians[1], radians[2]) )
        elif degrees[0] == 'q':
            if laser == 1:
                print "\nAdjusting LIDAR 2"
                laser = 2
                laser_frame = "laser2"
				parent_frame = "laser_mount2"
                current_angle_deg = [0, 0, 0]    
            elif laser == 2:
                break
            else:
				print "You have not given a valid Input"
        sleep(1)
    print "You have stopped making changes"
    print "Quiting the LiDAR Claibration Tool"
    sleep(1)
