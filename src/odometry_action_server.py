#! /usr/bin/env python
import rospy
import actionlib
from nav_msgs.msg import Odometry
from odometry_topic_subscriber import OdomSubClass
from my_turtlebot_topics.msg import record_odomFeedback, record_odomResult, record_odomAction
from geometry_msgs.msg import Vector3
import math 

class OdometeryActionServerClass(object):
    def __init__(self,goal_distance):
         # creates the action server
        self._as = actionlib.SimpleActionServer("/odom_action_srv", record_odomAction, self.goal_callback, False)
        self._as.start()
        self._result = record_odomResult()
        self.odom_subs_object = OdomSubClass()
        self._seconds_recording = 120
        self._goal_distance = goal_distance

    def goal_callback(self,goal):
        # helper variables
        r = rospy.Rate(1)
        success = True   
        for i in range(self._seconds_recording):
            # check that preempt (cancelation) has not been requested by the action client
            if self._as.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                # the following line, sets the client in preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                break
      
            if not self.goal_reached():
                rospy.logdebug('Reading Odometry...')
                self._result.result_odom_array.append(self.odom_subs_object.get_odomdata())
            else:
                rospy.logwarn('Reached distance Goal')
            
            r.sleep()
        
        # at this point, either the goal has been achieved (success==true)
        # or the client preempted the goal (success==false)
        # If success, then we publish the final result
        # If not success, we do not publish anything in the result
        if success:
           self._as.set_succeeded(self._result)

        self.clean_variables()
    
    def clean_variables(self):
         self._result = record_odomResult()
    
    def goal_reached(self):
        distance = self.get_distance_moved()
        rospy.loginfo("Distance Moved="+str(distance))
        return distance > self._goal_distance

    #returns distance between the two points a a vector
    def create_vector(self, p1, p2):
        
        distance_vector = Vector3()
        distance_vector.x = p2.x - p1.x
        distance_vector.y = p2.y - p1.y
        distance_vector.z = p2.z - p1.z
        
        return distance_vector
    
    #get tehe distance of the odometry data between the goal and the start position
    def get_distance_moved(self):
        
        distance = float('-inf') 
        data = self._result.result_odom_array
        if len(data) >= 2 :
            
            start =  data[0].pose.pose.position
            end = data[len(data)-1].pose.pose.position
            
            rospy.loginfo("start_position ==>"+str(start))
            rospy.loginfo("end_position ==>"+str(end))
            
             
            distance_vector = self.create_vector(start, end)
            rospy.loginfo("Distance Vector ==>"+str(end))
            
            distance = abs(distance_vector.y)
            rospy.loginfo("Distance ==>"+str(distance))
        
        else:
            rospy.logerr("Odom array doesnt have the minimum number of elements = "+str(len(data)))
        
        return distance
      
if __name__ == '__main__':
  rospy.init_node('odom_action_server_node')
  OdometeryActionServerClass(goal_distance=8.77)
  rospy.spin()
