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
        #Initatie the MarkerArray
        self.myMarkerArray = MarkerArray()
        self.myMarkerArray.markers = []
        
        #Initiate Transform Buffer
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        #Initate OccupancyGrid Publisher
        self.pub = rospy.Publisher('MarkerArray', MarkerArray, queue_size=1)
        
        #Initiate Point Cloud Subscriber
        self.sub = rospy.Subscriber('OccGrid', OccupancyGrid, self.callback)

    def callback(self, occupancygrid):
        t1 = time.time()
        heightGrid = np.array(occupancygrid.data)
        binaryGrid = (heightGrid > 0.2).astype(np.int_)
        binaryGrid = np.reshape(np.flip(binaryGrid,0), (int(occupancygrid.info.height), int(occupancygrid.info.width))).T
        rospy.loginfo(binaryGrid)
        
        structure = np.ones((3, 3), dtype=np.int)
        labeled, ncomponents = label(binaryGrid, structure)
        indices = np.indices(binaryGrid.shape).T[:,:,[1, 0]]
        
        rospy.loginfo(ncomponents)
        
        rospy.loginfo('t2: '+str(time.time()-t1))
        #1. connected components
        #2. for each cc make a marker
        #3. put marker in MarkerArray
        
        #self.pub.publish()
        
    #def get_con_comp(self, grid):
        #blah

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
        marker.color.a = 0.5
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.mesh_resource = ""
        
        return marker

if __name__ == '__main__':
    #Initiate the Node
    rospy.init_node('object_detection', anonymous=True)
    
    #Initiate Subscriber and Publisher object
    a = SubscribeAndPublish()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
