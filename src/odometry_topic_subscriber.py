#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry 

class OdomSubClass(object):
    def __init__(self):
        #create a subscriber
        self._odom_sub=rospy.Subscriber('/odom',Odometry,self.class_callback)
        #subscriber message type object
        self._odom_msg_object = Odometry()

    #callback function for the subscriber
    def class_callback(self,msg):
        #get the message in laserscan structure
        self._odom_msg_object = msg
        #log the data in debug mode
        rospy.logdebug(self._odom_msg_object)
    
    #returns the laser data
    def get_odomdata(self):
        return self._odom_msg_object


if __name__ == "__main__":
    rospy.init_node('odom_sub_node',log_level=rospy.INFO)
    odom_sub_object = OdomSubClass()
    rate = rospy.Rate(1)
    ctrl_c = False

    def shutdownhook():
        global ctrl_c
        # works better than the rospy.is_shut_down()
        print("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        data = odom_sub_object.get_odomdata()
        rospy.loginfo(data)
        rate.sleep()

    
