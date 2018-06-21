#! /usr/bin/env python

#####################################################################
# NODE DETAILS:                                                     #
# This node recives point cloud data from the LIDAR and creates a   #
# grid that illustrates where in 2D space an object may be          #
# occupying space. The values in the grid is determined by the      #
# heighest point in that space.                                     #
#                                                                   #
# PARAMETERS:                                                       #
#     - laser_frame:Defines the name of the frame where the LIDAR   #
#                       data is broadcast to.                       #
#     - grid_frame: Defines the name of the frame that the          #
#                       occupancy grid is in.                       #
#     - grid_height: Defines the height of the occupancy grid       #
#     - grid_width: Defines the width of the occupancy gird         #
#     - grid_x: Defines the x displacement of the occupancy grid    #
#     - grid_y: Defines the y displacement of the occupancy grid    #
#                                                                   #
# TOPICS:                                                           #
#    SUBSCRIBED:                                                    #
#        - cloud_drop                                               #
#    PUBLISHED:                                                     #
#        - occupancy_grid                                           #
#####################################################################

import struct
import rospy
import tf2_ros
import PyKDL
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

# This class manages the subscriber and publisher
# for this node.
class SubscribeAndPublish(object):
    def __init__(self, ns, laser_frame, grid_frame, grid_width, grid_height, grid_x, grid_y):
        #Store Namespace
        self.ns = ns[1]
        self.laser_frame = laser_frame

        #Initatie the occupancyGrid
        self.myOccupancyGrid = OccupancyGrid()
        self.myOccupancyGrid.header.frame_id = grid_frame
        self.myOccupancyGrid.info.resolution = 0.1
        self.myOccupancyGrid.info.width = grid_width/self.myOccupancyGrid.info.resolution #x
        self.myOccupancyGrid.info.height = grid_height/self.myOccupancyGrid.info.resolution #y
        self.myOccupancyGrid.info.origin.position.x = grid_x
        self.myOccupancyGrid.info.origin.position.y = grid_y
        self.myOccupancyGrid.info.origin.position.z = 0
        self.myOccupancyGrid.info.origin.orientation.x = 0
        self.myOccupancyGrid.info.origin.orientation.y = 0
        self.myOccupancyGrid.info.origin.orientation.z = 0
        self.myOccupancyGrid.info.origin.orientation.w = 1
        
        #Initiate Transform Buffer and Listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #Initate OccupancyGrid Publisher
        self.pub = rospy.Publisher('occupancy_grid', OccupancyGrid, queue_size=1)
        
        #Initiate Point Cloud Subscriber
        self.sub = rospy.Subscriber('cloud_drop', PointCloud2, self.callback)
    
    def callback(self, cloud):
        #Recieve transform from buffer between laser and map
        try:
            t = self.tfBuffer.lookup_transform(self.myOccupancyGrid.header.frame_id, self.laser_frame, rospy.Time(0))
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
        
        #Read points from point cloud
        transformedMapPoints = self.readAndTransformPoints(cloud, tran)
        #Transform points from LIDAR coordinates to Grid coordinates
        transformedGridPoints = transformedMapPoints - (self.myOccupancyGrid.info.origin.position.x, self.myOccupancyGrid.info.origin.position.y, 0, 0)
        transformedGridPoints = transformedGridPoints/self.myOccupancyGrid.info.resolution
        transformedGridPoints = np.floor(transformedGridPoints)
        #For every transformed point inside the grid
        for point in transformedGridPoints:
            if point[0] >= 0 and point[0] < self.myOccupancyGrid.info.width and point[1] >= 0 and point[1] < self.myOccupancyGrid.info.height:
                #Set the square where the transformed point is closest, 
                # to the height of the point in that square.
                index = int(self.myOccupancyGrid.info.width*point[1] + point[0])
                if point[2] > gridData[index]:
                    gridData[index] = point[2]
        
        #Update OccupancyGrid  
        self.myOccupancyGrid.data = gridData
        
        #Publish OccupancyGrid
        self.pub.publish(self.myOccupancyGrid)


    #Read points from cloud and transform into map frame
    def readAndTransformPoints(self, cloud, tran):
        width, height, point_step, row_step, data = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data
        #Binary format for one point [float, float, float, float]
        fmt="ffff"
        unpack_from = struct.Struct(fmt).unpack_from
        a = np.empty((22176,4))
        index = 0
        if height > 24:
            height =24
        #Loop through all 24 layers
        for v in range(height):
                offset = row_step * v
                for u in range(width):
                    #Unpack next point and convert it from binary
                    p = unpack_from(data, offset)
                    a[index] = np.matmul(tran, [p[0], p[1], p[2], 1])   
                    offset += point_step
                    index += 1
        return a

if __name__ == '__main__':
    #Initiate the Node
    rospy.init_node('collision_detection')

    #Get ROS Parameters
    ns = rospy.get_namespace()
    laser_frame = rospy.get_param('~laser_frame')
    grid_frame = rospy.get_param('~grid_frame')
    grid_height = rospy.get_param('~grid_height')
    grid_width = rospy.get_param('~grid_width')
    grid_x = rospy.get_param('~grid_x')
    grid_y = rospy.get_param('~grid_y')

    #Initiate object
    a = SubscribeAndPublish(ns, laser_frame, grid_frame, grid_width, grid_height, grid_x, grid_y)

    #Hold node open until ROS system is shutdown
    rospy.spin()
