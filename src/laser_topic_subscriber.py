#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

class LaserSubClass(object):
    def __init__(self):
        #create a subscriber
        self._laser_sub=rospy.Subscriber('/kobuki/laser/scan',LaserScan,self.class_callback)
        #subscriber message type object
        self._laser_msg_object = LaserScan()

    #callback function for the subscriber
    def class_callback(self,msg):
        #get the message in laserscan structure
        self._laser_msg_object = msg
        #log the data in debug mode
        rospy.logdebug(self._laser_msg_object)
    
    #returns the laser data
    def get_laserdata(self):
        return self._laser_msg_object


if __name__ == "__main__":
    rospy.init_node('laser_sub_node',log_level=rospy.INFO)
    laser_sub_object = LaserSubClass()
    rate = rospy.Rate(1)
    ctrl_c = False

    def shutdownhook():
        global ctrl_c
        # works better than the rospy.is_shut_down()
        print("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        data = laser_sub_object.get_laserdata()
        rospy.loginfo(data)
        rate.sleep()

    
