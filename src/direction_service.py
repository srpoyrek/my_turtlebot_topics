#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse # you import the service message python classes generated from Trigger.srv.
import time
from laser_topic_subscriber import LaserSubClass 

class DirectionServiceClass(object):
    def __init__(self): 
        self._dir_srv = rospy.Service('/crash_direction_service', Trigger , self.class_callback) # create the Service called crash_direction_service with the defined callback and message structure Trigger
        self._laser_sub = LaserSubClass() # laser subcriber topic object which reads the laser data
        self.dirc_data ={"front":0.0,"left":0.0,"right":0.0} # intialize a dict to store the front left and right laser data readings

    # callback for the service
    def class_callback(self,request):
        #get the crash data
        self.dirc_data = self._laser_sub.get_crash_data()
        #create a response 
        response = TriggerResponse()
        response.success = self.potentional_crash() # success is the front crash flag
        response.message = self.direction_to_move() # right or left indicator
        return response # the service Response class, in this case EmptyResponse
    
    #if  front laser reading less than 0.8 indicates potential crash
    def potentional_crash(self):
        if self.dirc_data["front"] < 0.8: #if less than 0.8 indicates potential crash
            return False
        else:
            return True
    #if right laser readings is more than left move right else move left 
    def direction_to_move(self):
        if self.dirc_data["right"] > self.dirc_data["left"]:
            return "right"
        else:
            return "left"
    

if __name__=="__main__":
    rospy.init_node('direction_service_srv_node')
    dir_srv_object = DirectionServiceClass()
    rospy.spin() # maintain the service open.
