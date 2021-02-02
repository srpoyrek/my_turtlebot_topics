#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class CmdVelPubClass(object):
    def __init__(self):
        #create a publisher
        self._cmd_vel_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #publisher message object
        self._twist_msg_object = Twist()
        # gemometry message linear and angular speed variables for /cmd_vel message
        self.rate = rospy.Rate(1)
        self.linearspeerd = 0.5
        self.angluarspeed = 0.25

    
    def publish_once_in_cmd_vel(self, cmd):
        """
        This is because publishing in topics sometimes fails teh first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        connections = self._cmd_vel_pub.get_num_connections()
        if connections > 0:
            self._cmd_vel_pub.publish(cmd)
            rospy.loginfo("Publish in cmd_vel...")
        else:
            self.rate.sleep()
    
    #move the robot with a given direction
    def move_robot(self,direction):
        if direction =="Forward":
            self._twist_msg_object.linear.x = self.linearspeerd
            self._twist_msg_object.angular.z = 0

        elif direction =="left":
            self._twist_msg_object.linear.x = 0
            self._twist_msg_object.angular.z = -self.angluarspeed            

        elif direction =="right":
            self._twist_msg_object.linear.x = 0
            self._twist_msg_object.angular.z = self.angluarspeed
        
        elif direction=="backward":
            self._twist_msg_object.linear.x = - self.linearspeerd 
            self._twist_msg_object.angular.z = 0
        
        elif direction=="Stop":
            self._twist_msg_object.linear.x = 0
            self._twist_msg_object.angular.z = 0

        else:
            pass
        #publish the message once in the /cmd_vel topic
        self.publish_once_in_cmd_vel(self._twist_msg_object)




if __name__ == "__main__":
    rospy.init_node('cmd_vel_pub_node')
    cmd_vel_pub_object = CmdVelPubClass()
    rate = rospy.Rate(1)
    ctrl_c = False

    def shutdownhook():
        global ctrl_c
        # works better than the rospy.is_shut_down()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
        cmd_vel_pub_object.move_robot(direction="Stop")
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        cmd_vel_pub_object.move_robot(direction="Forward")
        rate.sleep()

    







