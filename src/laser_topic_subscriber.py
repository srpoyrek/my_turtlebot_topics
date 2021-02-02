#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import time

class LaserSubClass(object):
    def __init__(self):
        #create a subscriber
        self._laser_sub=rospy.Subscriber('/kobuki/laser/scan',LaserScan,self.class_callback)
        #subscriber message type object
        self._laser_msg_object = LaserScan()
        self.front = 0.0
        self.right = 0.0
        self.left = 0.0

    #callback function for the subscriber
    def class_callback(self,msg):
        #get the message in laserscan structure
        self._laser_msg_object = msg
        #log the data in debug mode
        rospy.logdebug(self._laser_msg_object)
    
    #returns the laser data
    def get_laserdata(self):
        return self._laser_msg_object
    
    #gets data left right and front laser readings
    def crash_data(self):
        self.front = self._laser_msg_object.ranges[360]
        self.right = self._laser_msg_object.ranges[0]
        self.left = self._laser_msg_object.ranges[719]
        rospy.loginfo("Distance in front = "+str(self.front))
        rospy.loginfo("Distance in right = "+str(self.left))
        rospy.loginfo("Distance in left = "+str(self.right))
    
    #returns crash data as dict
    def get_crash_data(self):
        return {"front": self.front, "left":self.left, "right":self.right}

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
    
    time.sleep(2)
    while not ctrl_c:
        data = laser_sub_object.get_laserdata()
        laser_sub_object.crash_data()
        #rospy.loginfo(data.ranges)