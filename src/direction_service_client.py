#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest # you import the service message python classes generated from Trigger.srv.
import sys


# Initialise a ROS node with the name service_client
rospy.init_node('direction_service_client_node')

# Wait for the service client /crash_direction_service to be running
rospy.wait_for_service('/crash_direction_service')

# Create the connection to the service
dir_srv_cleint = rospy.ServiceProxy('/crash_direction_service', Trigger)

# Create an msg request object
dir_srv_cleint_msg_rqst = TriggerRequest()
rate = rospy.Rate(1)
ctrl_c = False
def shutdownhook():
    global ctrl_c
    # works better than the rospy.is_shut_down()
    print("shutdown time!")
    ctrl_c = True
    
rospy.on_shutdown(shutdownhook)

while not ctrl_c:
    result = dir_srv_cleint(dir_srv_cleint_msg_rqst) # get the direction from the direction client

    if result.success:
        #if true meaning potential collision in front
        rospy.logwarn("Success =="+str(result.success)) # print the result given by the service called
        rospy.logwarn("Direction To Go=="+str(result.message)) # print the result given by the service called
    else:
        #else show the direction to move
        rospy.logwarn("Success =="+str(result.success)) # print the result given by the service called
        rospy.logwarn("Direction To Go=="+str(result.message)) # print the result given by the service called

    rate.sleep()

