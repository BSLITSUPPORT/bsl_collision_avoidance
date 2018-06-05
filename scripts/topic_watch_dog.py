#!/usr/bin/env python

#####################################################################
# NODE DETAILS:                                                     #
# This node monitors a topic and determines if it is recieving      #
# data. If no data is recieved from the topic in the time defined   #
# by the timeout parameter, it publishes that the topic has failed. #
#                                                                   #
# PARAMETERS:                                                       #
#     - topic:         Defines which topic to Monitor.              #
#    - timeout:        Defines the period of time where no messages #
#                        are recieved before a failure is noticed.  #
#                                                                   #
# TOPICS:                                                           #
#    SUBSCRIBED:                                                    #
#        - Given by the topic parameter 'topic'                     #
#    PUBLISHED:                                                     #
#        - failures                                                 #
#####################################################################

import rospy
from rospy import sleep
from bsl_collision_avoidance.msg import TopicState

# This class manages the subscriber and publisher
# for this node.
class SubscribeAndPublish(object):
    def __init__(self, topic, timeout):
        #Initate failures publisher
        pub = rospy.Publisher('failures', TopicState, queue_size=1)
        
        #Initiate Subscriber 
        rospy.Subscriber(topic, rospy.AnyMsg, self.callback)
        
        self.triggered = rospy.get_time()
        while not rospy.is_shutdown():
            #If topic has timed out and no message has been recieved
            if rospy.get_time() - self.triggered >= timeout:
                #Then publish a message saying that topic is no longer alive
                failureMsg = TopicState()
                failureMsg.topic_name = rospy.get_namespace()+topic
                failureMsg.is_alive = 0
                pub.publish(failureMsg)
            #If topic has not timed out
            else:
                #Then publish a message saying that topic is alive
                successMsg = TopicState()
                successMsg.topic_name = rospy.get_namespace()+topic
                successMsg.is_alive = 1
                pub.publish(successMsg)
            #Sleep for a third of the timeout period
            sleep(timeout/3)
            
    
    def callback(self, cloud):
        #Record time when a message was recieved from a topic
        self.triggered = rospy.get_time()
        

if __name__ == '__main__':
    #Initialise node
    rospy.init_node('topic_watch_dog')
    
    #Get parameters
    topic = rospy.get_param('~topic')
    timeout = rospy.get_param('~timeout')
    
    #Subscribe
    a = SubscribeAndPublish(topic, timeout)
    
    #Hold node open until ROS system is shutdown
    rospy.spin()
    
    
    
