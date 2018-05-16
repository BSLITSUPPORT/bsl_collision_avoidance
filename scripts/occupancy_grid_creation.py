#! /usr/bin/env python
import struct
import rospy
import tf2_ros
import PyKDL
import time
import numpy as np
from numba import *
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from tf2_kdl.tf2_kdl import transform_to_kdl

class SubscribeAndPublish(object):
    def __init__(self):        
        #Initatie the occupancyGrid
        self.myOccupancyGrid = OccupancyGrid()
        self.myOccupancyGrid.header.frame_id = "map"
        self.myOccupancyGrid.info.resolution = 0.1
        self.myOccupancyGrid.info.width = 30/self.myOccupancyGrid.info.resolution #x
        self.myOccupancyGrid.info.height = 3.5/self.myOccupancyGrid.info.resolution #y
        self.myOccupancyGrid.info.origin.position.x = 5.5
        self.myOccupancyGrid.info.origin.position.y = -1.5
        self.myOccupancyGrid.info.origin.position.z = 0
        self.myOccupancyGrid.info.origin.orientation.x = 0
        self.myOccupancyGrid.info.origin.orientation.y = 0
        self.myOccupancyGrid.info.origin.orientation.z = 0
        self.myOccupancyGrid.info.origin.orientation.w = 0
        
        #Find Transform
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        #Initate OccupancyGrid Publisher
        self.pub = rospy.Publisher('OccGrid', OccupancyGrid, queue_size=1)
        
        #Initiate Point Cloud Subscriber
        self.sub = rospy.Subscriber(str(rospy.get_param('~cloud_topic_name')), PointCloud2, self.callback)
    
    def callback(self, cloud):
        t1 = time.time()
        #Recieve transform from buffer between laser and map
        try:
            t = self.tfBuffer.lookup_transform('map', 'laser', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        
        #Convert ROS transform to 
        self.myOccupancyGrid.header.stamp = cloud.header.stamp
        self.myOccupancyGrid.info.map_load_time = cloud.header.stamp
        
        #Initiate empty array
        gridData = np.zeros(int(self.myOccupancyGrid.info.width*self.myOccupancyGrid.info.height))
        
        #Get transformation matrix from ROS transform
        tran = np.identity(4)
        rot = PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
        for i in range(3):
            for j in range(3):
                tran[i, j] = rot[i, j]
        tran[0, 3] = t.transform.translation.x
        tran[1, 3] = t.transform.translation.y
        tran[2, 3] = t.transform.translation.z
        
        #
        transformedMapPoints = self.readAndTransformPoints(cloud, tran)
        transformedGridPoints = transformedMapPoints - (self.myOccupancyGrid.info.origin.position.x, self.myOccupancyGrid.info.origin.position.y, 0, 0)
        transformedGridPoints = transformedGridPoints/self.myOccupancyGrid.info.resolution
        transformedGridPoints = np.floor(transformedGridPoints)
        for point in transformedGridPoints:
            if point[0] >= 0 and point[0] < self.myOccupancyGrid.info.width and point[1] >= 0 and point[1] < self.myOccupancyGrid.info.height:
                index = int(self.myOccupancyGrid.info.width*point[1] + point[0])
                if point[2] > gridData[index]:
                    gridData[index] = point[2]
        
        #Update OccupancyGrid  
        self.myOccupancyGrid.data = gridData
        
        #Publish OccupancyGrid
        self.pub.publish(self.myOccupancyGrid)
        
        rospy.loginfo('t1: '+str(time.time()-t1))
    
    #Read points from cloud and transform into map frame
    @jit
    def readAndTransformPoints(self, cloud, tran):
        fmt="ffff"
        width, height, point_step, row_step, data = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data
        unpack_from = struct.Struct(fmt).unpack_from
        a = np.empty((22176,4))
        index = 0
        for v in range(24):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    a[index] = np.matmul(tran, [p[0], p[1], p[2], 1])
                    offset += point_step
                    index += 1
        return a

if __name__ == '__main__':
    #Initiate the Node
    rospy.init_node('collision_detection', anonymous=True)
    
    #Initiate object
    a = SubscribeAndPublish()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
