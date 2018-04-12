#!/usr/bin/env /usr/bin/python

import rospy
import tf2_ros
import time
import numpy as np
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from scipy.ndimage.measurements import label

class SubscribeAndPublish:
    def __init__(self):
        #Initiate Transform Buffer
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        #Initate OccupancyGrid Publisher
        self.pub = rospy.Publisher('MarkerArray', MarkerArray, queue_size=1)
        
        #Initiate Point Cloud Subscriber
        self.sub = rospy.Subscriber('OccGrid', OccupancyGrid, self.callback)

    def callback(self, occupancygrid):
        t1 = time.time()
        
        #Initatie the MarkerArray
        self.myMarkerArray = MarkerArray()
        self.myMarkerArray.markers = []
        
        self.heightGrid = np.reshape(np.array(occupancygrid.data), (int(occupancygrid.info.height), int(occupancygrid.info.width))).T
        self.binaryGrid = (self.heightGrid > 0.2).astype(np.int_)
        
        rospy.loginfo(self.binaryGrid.shape)
        
        structure = np.ones((3, 3), dtype=np.int)
        labeled, ncomponents = label(self.binaryGrid, structure)
        indices = np.indices(self.binaryGrid.shape).T[:,:,[0, 1]]
        
        for i in range(ncomponents):
            i += 1
            position, size = self.ccProperties(indices[(labeled == i).T])
            position = position*occupancygrid.info.resolution #Convert grid index to m
            size = size*occupancygrid.info.resolution #Convert grid index to m
            position = position + (occupancygrid.info.origin.position.x, occupancygrid.info.origin.position.y, 0) #Correct Map to Grid displacment
            self.myMarkerArray.markers.append(self.cube(i-1, position, size))
        
        rospy.loginfo('t2: '+str(time.time()-t1))
        #1. connected components
        #2. for each cc make a marker
        #3. put marker in MarkerArray
        
        self.pub.publish(self.myMarkerArray)

    def cube(self, idnum, position, size):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.ns = "shape_namespace"
        marker.id = idnum
        marker.type = 1
        marker.action = 0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]
        marker.color.a = 0.9
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.lifetime = rospy.Duration(0.4) 
        marker.mesh_resource = ""
        
        return marker
        
    def ccProperties(self, indices):
        #x axis
        top = indices[0,0]
        bottom = indices[0,0]
        #y axis
        left = indices[0,1]
        right = indices[0,1]
        #z axis
        height = 0
        
        for index in indices:
            if index[0] > top: top = index[0]
            elif index[0] < bottom: bottom = index[0]
            if index[1] < right: right = index[1]
            elif index[1] > left: left = index[1]
            if self.heightGrid[index[0]][index[1]] > height: height = self.heightGrid[index[0]][index[1]]
        
        position = np.array([(top+1+bottom)/2, (left+1+right)/2, height/2])
        size = np.array([top+1-bottom, left+1-right, height+1])

        return position, size

if __name__ == '__main__':
    #Initiate the Node
    rospy.init_node('object_detection', anonymous=True)
    
    #Initiate Subscriber and Publisher object
    a = SubscribeAndPublish()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
