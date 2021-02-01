#! /usr/bin/env python
import rospy
import time
import actionlib
from nav_msgs.msg import Odometry
from my_turtlebot_topics.msg import record_odomGoal, record_odomFeedback, record_odomResult, record_odomAction

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

def feedback_callback(feedback):
    rospy.loginfo("Rec Odom Feedback feedback ==>"+str(feedback))


# initializes the action client node
rospy.init_node('odom_action_client_node')

# create the connection to the action server
client = actionlib.SimpleActionClient('/odom_action_srv', record_odomAction)
# waits until the action server is up and running
client.wait_for_server()

# creates a goal to send to the action server
goal = record_odomGoal()
client.send_goal(goal, feedback_cb=feedback_callback)


state_result = client.get_state()

rate = rospy.Rate(1)

rospy.loginfo("state_result: "+str(state_result))

while state_result < DONE:
    rospy.loginfo("Doing Stuff while waiting for the Server to give a result....")
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))
    
rospy.loginfo("[Result] State: "+str(state_result))
if state_result == ERROR:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("There is a warning in the Server Side")

rospy.loginfo("[Result] State: "+str(client.get_result()))