#!/usr/bin/env python

#####################################################################
# NODE DETAILS:                                                     #
# This node recieves all the messages from every topic watchdog     #
# and determines if a failure has occured. If so it sets the fault  #
# pin to LOW and doesn't switch it back until 2 seconds after the   #
# system has been restored.                                         #
#                                                                   #
# PARAMETERS:                                                       #
#     - none                                                        #
#                                                                   #
# TOPICS:                                                           #
#    SUBSCRIBED:                                                    #
#        - /1/failures                                              #
#        - /2/failures                                              #
#    PUBLISHED:                                                     #
#        - none                                                     #
#####################################################################

import rospy
from rospy import sleep
from pyModbusTCP.client import ModbusClient
from bsl_collision_avoidance.msg import TopicState

class Subscriber:
    def __init__(self, modbus_address):
        self.failureTimes = {}
        faultTime = rospy.get_time()
        
        #Initialise communication with ModbusClient
        adam = ModbusClient(host="192.168.1.3", port=502, auto_open=True, auto_close=False)
        
        rospy.Subscriber("/1/failures", TopicState, self.callback)
        rospy.Subscriber("/2/failures", TopicState, self.callback)
        
        #While System has not been shutdown
        while not rospy.is_shutdown():            
            #If for some reason the watchdogs or subscription fail tell PLCs
            if len(self.failureTimes.keys()) == 0:
                adam.write_single_coil(int(modbus_address), False)
            #If watch dogs are functioning 
            else:
                #Check current state of every topic
                for key in self.failureTimes.keys():
                    #If there is a failure in one of the topics
                    if self.failureTimes[key] == 0:
                        #Set pin to LOW
                        adam.write_single_coil(int(modbus_address), False)
                        faultTime = rospy.get_time()
                        print "fault"
                #If there has been no fault for 2 secs set pin back to HIGH
                if rospy.get_time() - faultTime >= 2:
                    adam.write_single_coil(int(modbus_address), True)
                    print "no fault"
            sleep(0.2)
        
    def callback(self, msg):
        #Store the current state of the topic against the topic name
        self.failureTimes[msg.topic_name] = msg.is_alive

    
if __name__ == '__main__':
    #Initiate Node
    rospy.init_node('failure_output')
    
    #Wait until all topics have been initialised
    sleep(6)
    
    #Get ROS parameters
    modbus_address = rospy.get_param('~modbus_address')
    
    #Start subscriber class
    a = Subscriber(modbus_address)
    
    #Hold node open until ROS system is shutdown
    rospy.spin()
