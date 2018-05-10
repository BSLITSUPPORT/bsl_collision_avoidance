#!/usr/bin/env /usr/bin/python

import rospy
import time
import numpy as np
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from scipy.ndimage.measurements import label

class SubscribeAndPublish:
    def __init__(self):
        #Initate OccupancyGrid Publisher
        self.pub = rospy.Publisher('MarkerArray', MarkerArray, queue_size=1)
        #Initiate Point Cloud Subscriber
        self.sub = rospy.Subscriber('OccGrid', OccupancyGrid, self.callback)
        
    def callback(self, occupancygrid):
        t2 = time.time()
        
        #Initatie the MarkerArray
        self.myMarkerArray = MarkerArray()
        self.myMarkerArray.markers = []
        #Recieve and reshape occupancy grid containing heights of all objects
        self.heightGrid = np.reshape(np.array(occupancygrid.data), (int(occupancygrid.info.height), int(occupancygrid.info.width))).T
        #Binarize the occupancy grid so all grid point above a certain height = 1 and other = 0
        self.binaryGrid = (self.heightGrid > 0.2).astype(np.int_)
        #Use scipy.ndimage.measurements.label to complete a connected component algorithim
        structure = np.ones((3, 3), dtype=np.int)
        labeled, ncomponents = label(self.binaryGrid, structure)
        indices = np.indices(self.binaryGrid.shape).T[:,:,[0, 1]]
        
        #Loop through make indices list a CC
        cclist = []
        for i in range(ncomponents):
            i+=1
            cclist.append(ConnectedComponent(i, self.heightGrid, indices[(labeled == i).T], occupancygrid.info.resolution, occupancygrid.info.origin.position.x, occupancygrid.info.origin.position.y))
            
        #Loop through list and join CCs that are inside each other.
        newcclist = []
        for i, cc in enumerate(cclist):
            if cc != 0:
                cclist[i] = 0
                #Loop through other cc in cclist
                for j, cc2 in enumerate(cclist[i:]):
                    if cc2 != 0:
                        if cc.inside(cc2):
                            cc.join(cc2)
                            cclist[i+j] = 0
                #Loop through new cc in new cc list
                broken = False
                for j, cc2 in enumerate(newcclist):
                    if cc2.inside(cc):
                        cc2.join(cc)
                        broken = True
                        break
                if broken == False:
                    newcclist.append(cc)
        
        for cc in newcclist:
            self.myMarkerArray.markers.append(cc.getMarker())

        rospy.loginfo('t2: '+str(time.time()-t2))
        
        #Publish MarkerArray
        self.pub.publish(self.myMarkerArray)

class ConnectedComponent:
    #Initialize class
    def __init__(self, idnum, heightGrid, indices, resolution, xDisplacment, yDisplacment):
        self.idnum = idnum
        self.position, self.size, self.bounds, self.corners = self.__ccProperties(heightGrid, indices, resolution, xDisplacment, yDisplacment)
    
    #Get properties of cc
    def __ccProperties(self, heightGrid, indices, resolution, xDisplacment, yDisplacment):
        #Setup initial bounds
        top = indices[0,0]
        bottom = indices[0,0]
        left = indices[0,1]
        right = indices[0,1]
        height = 0
        #Loop through all indices and find bounds of connected component 
        for index in indices:
            if index[0] > top: top = index[0]
            elif index[0] < bottom: bottom = index[0]
            if index[1] < right: right = index[1]
            elif index[1] > left: left = index[1]
            if heightGrid[index[0]][index[1]] > height: height = heightGrid[index[0]][index[1]]
        #Apply transform from grid to Map
        xMax = (top+1) * resolution + xDisplacment
        xMin = bottom * resolution + xDisplacment
        yMax = (left+1) * resolution + yDisplacment
        yMin = right * resolution + yDisplacment
        height = height * resolution
        #Return corners bounds position and size of connected component
        position = np.array([(xMax+xMin)/2, (yMax+yMin)/2, height/2])
        size = np.array([xMax-xMin, yMax-yMin, height])
        bounds = np.array([xMax, xMin, yMax, yMin, height])
        corners = np.array([[xMax, yMax], [xMax, yMin], [xMin, yMax], [xMin, yMin]])
        return position, size, bounds, corners
        
    # Checks if one of the corners of another connected component exists
    # within the the bounds of this connected component
    def inside(self, cc):
        corners = cc.getCorners()
        bounds = self.bounds
        for corner in corners:            
            if corner[0] <= bounds[0] and corner[0] >= bounds[1] and corner[1] <= bounds[2] and corner[1] >= bounds[3]:
                return True
        return False
    
    # Joins two connected components into this one
    def join(self, cc):
        bounds = cc.getBounds()
        if bounds[0] > self.bounds[0]: self.bounds[0] = bounds[0]
        if bounds[1] < self.bounds[1]: self.bounds[1] = bounds[1]
        if bounds[2] > self.bounds[2]: self.bounds[2] = bounds[2]
        if bounds[3] < self.bounds[3]: self.bounds[3] = bounds[3]
        if bounds[4] > self.bounds[4]: self.bounds[4] = bounds[4]
        if cc.getID() < self.idnum: self.idnum = cc.getID()
        self.recalculate()
    
    #Recalculates the different CC properties after changing the bounds
    def recalculate(self):
        xMax, xMin, yMax, yMin, height = self.bounds
        self.position = np.array([(xMax+xMin)/2, (yMax+yMin)/2, height/2])
        self.size = np.array([xMax-xMin, yMax-yMin, height])
        self.corners = np.array([[xMax, yMax], [xMax, yMin], [xMin, yMax], [xMin, yMin]])
        
    #Return corners of CC
    def getCorners(self):
        return self.corners
    
    #Return bounds of CC
    def getBounds(self):
        return self.bounds
    
    #Return ID number of CC
    def getID(self):
        return self.idnum
    
    #Function that creates and returns a Cube marker
    def getMarker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.ns = "shape_namespace"
        marker.id = self.idnum
        marker.type = 1
        marker.action = 0
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = self.size[0]
        marker.scale.y = self.size[1]
        marker.scale.z = self.size[2]
        marker.color.a = 0.9
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.lifetime = rospy.Duration(0.4) 
        marker.mesh_resource = ""
        return marker

if __name__ == '__main__':
    #Initiate the Node
    rospy.init_node('object_detection', anonymous=True)
    
    #Initiate Subscriber and Publisher object
    a = SubscribeAndPublish()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
