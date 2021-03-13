#! /usr/bin/env python 

import rospy
from geometry_msgs.msg import Twist

class conv_vel():
    def __init__(self):
        self.pub = rospy.Publisher('/twist_vels', Twist, queue_size=1)
        self.show = Twist()
        rospy.sleep(0.1)
    
    def show_vels(self):
        self.show.linear.x = -0.3
        self.show.angular.z = 0.3
        self.pub.publish(self.show)
        print(self.show)

if __name__=='__main__':
    rospy.init_node('publish_velo')
    x = conv_vel()
    x.show_vels()