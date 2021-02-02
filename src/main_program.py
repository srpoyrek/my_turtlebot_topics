#! /usr/bin/env python
import rospy
import time
import actionlib
from nav_msgs.msg import Odometry
from my_turtlebot_topics.msg import record_odomGoal, record_odomFeedback, record_odomResult, record_odomAction
from std_srvs.srv import Trigger, TriggerRequest # you import the service message python classes generated from Trigger.srv.
import sys
from cmd_vel_publisher import CmdVelPubClass
from odom_analysis import OdometryAnalysis
from odom_analysis import check_if_out_maze

class ControlTurtlebot(object):
    def __init__(self,goal_distance):
        self._goal_distance = goal_distance
        self.init_direction_service_client()
        self.init_rec_odom_action_client()
        self.init_move_turtlebot_publisher()

    #initialize the direction service client
    def init_direction_service_client(self):
        rospy.loginfo('Waiting for Service Server')
        # Wait for the service client /crash_direction_service to be running
        rospy.wait_for_service('/crash_direction_service')
        rospy.loginfo('Service Server Found...')
        # Create the connection to the service
        self._dir_srv_cleint = rospy.ServiceProxy('/crash_direction_service', Trigger)
        # Create an msg request object
        self._dir_srv_cleint_msg_rqst = TriggerRequest()
    
    #initialize the odometry action client
    def init_rec_odom_action_client(self):
        # create the connection to the action server
        self._odom_action_client = actionlib.SimpleActionClient('/odom_action_srv', record_odomAction)
        # waits until the action server is up and running
        rospy.loginfo('Waiting for Action Server')
        self._odom_action_client.wait_for_server()
        rospy.loginfo('Action Server Found...')
        # creates a goal to send to the action server
        self._odom_action_client_goal = record_odomGoal()
    
    #initialize the cmd _vel publisher
    def init_move_turtlebot_publisher(self):
        self._cmdvelpub_object = CmdVelPubClass()

    # get the direction to move from the direction service client
    def make_direction_request(self):
        result = self._dir_srv_cleint(self._dir_srv_cleint_msg_rqst) # send the name of the object to be deleted by the service through the connection
        return result

    #send_goal to the odometry action_server
    def send_goal_to_rec_odom_action_server(self):
        self._odom_action_client.send_goal(self._odom_action_client_goal, feedback_cb=self.rec_odom_feedback_callback)
    
    #feedback callback for the odometry action client
    def rec_odom_feedback_callback(self,feedback):
        rospy.loginfo("Rec Odom Feedback feedback ==>"+str(feedback))
    
    #returns the state of the odometry action 
    def rec_odom_finished(self):
        has_finished = ( self._odom_action_client.get_state() >= 2 )
        return has_finished
    
    #move turtle bot
    def move_turtlebot(self, direction):
        self._cmdvelpub_object.move_robot(direction)

    #returns the odometry action client results
    def get_result_rec_odom(self):
        return self._odom_action_client.get_result()

    def got_out_maze(self, odom_result_array):
        return check_if_out_maze(self._goal_distance, odom_result_array)

    

#initialize a main node
rospy.init_node("turtlebot_main_node", log_level=rospy.INFO)
control_turtlebot_object = ControlTurtlebot(goal_distance=8.77)
rate = rospy.Rate(10)
#send the goal to the action server
control_turtlebot_object.send_goal_to_rec_odom_action_server()
i = 0

while not control_turtlebot_object.rec_odom_finished():
    #get the direction to move
    direction_to_go = control_turtlebot_object.make_direction_request()
    # if success move forward else stop try again for more time for multiple times if still no response then stop

    if direction_to_go.success is True:
        control_turtlebot_object.move_turtlebot(direction="Forward")
    else:
        control_turtlebot_object.move_turtlebot(direction="Stop")
        time.sleep(2)
        control_turtlebot_object.move_turtlebot(direction_to_go.message)
        if i==0:
            time.sleep(6.1)
            
        elif i==1:
            time.sleep(6.15)
            
        elif i==2:
            time.sleep(7)
            
        else:
            time.sleep(6.2)
        
        i=i+1    
        control_turtlebot_object.move_turtlebot(direction="Stop")
        time.sleep(2)
        
    rate.sleep()
    


odom_result = control_turtlebot_object.get_result_rec_odom()
odom_result_array = odom_result.result_odom_array

if control_turtlebot_object.got_out_maze(odom_result_array):
    control_turtlebot_object.move_turtlebot(direction="Stop")
    time.sleep(2)
    rospy.loginfo("Out of Maze")
else:
    rospy.loginfo("In Maze")
    control_turtlebot_object.move_turtlebot(direction="Stop")
    time.sleep(2)

rospy.loginfo("Turtlebot Maze test Finished")

