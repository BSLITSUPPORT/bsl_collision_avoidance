#!/usr/bin/env python

#####################################################################
# NODE DETAILS:                                                     #
# When the system is initialised this node reads the saved LIDAR    #
# transform from it's file and broadcasts a Static Transform.       #
#                                                                   #
# PARAMETERS:                                                       #
#     - laser_frame: Defines the name of the frame where the LIDAR  #
#                        will be mounted.                           #
#    - parent_frame: Defines the name of the parent frame of the    #
#                        LIDAR                                      #
#                                                                   #
# TOPICS:                                                           #
#    SUBSCRIBED:                                                    #
#        - none                                                     #
#    PUBLISHED:                                                     #
#        - tf_static                                                #
#####################################################################

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
import rospkg


def transform_broadcaster(child_frame, parent_frame, x, y, z, height):
    #Initalises Static Broadcaster
    br = tf2_ros.StaticTransformBroadcaster()
    
    #Create Transform
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
    
    #Broadcast Transform
    br.sendTransform(t)
    
if __name__ == '__main__':
    #Initiate Node
    rospy.init_node('initialise_LIDAR_transform')

    #Get ROS Paramaters
    child_frame = rospy.get_param('~laser_frame')
    parent_frame = rospy.get_param('~parent_frame')

    #Locate file
    rospack = rospkg.RosPack()
    path = rospack.get_path('bsl_collision_avoidance')

    #Read angles from file
    f = open(path+'/config/'+child_frame+'.txt', 'r')
    laserangle = map(float, f.readline().split(', '))
    f.close()

    #Broadcast Transform from files
    transform_broadcaster(child_frame, parent_frame, laserangle[0], laserangle[1], laserangle[2], laserangle[3])
    
    #Hold node open until ROS system is shutdown
    rospy.spin()
