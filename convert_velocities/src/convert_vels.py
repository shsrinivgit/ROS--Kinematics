#! /usr/bin/env python 

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class read_convert():
    def __init__(self):
        self.sub = rospy.Subscriber('/twist_vels', Twist, self.my_callback)
        self.pub1 = rospy.Publisher('/left_wheel_controller/command', Float64, queue_size=1)
        self.pub2 = rospy.Publisher('/right_wheel_controller/command', Float64, queue_size = 1)
        self.mov = Twist()
        self.Vr = Float64()
        self.Vl = Float64()
        rospy.sleep(0.5)
    
    def my_callback(self, msg):
        self.mov = msg

    def convert_vel(self):
        self.Vr = ((2*self.mov.linear.x) + (self.mov.angular.z * 0.3))/(2*0.1)
        self.Vl = ((2*self.mov.linear.x) - (self.mov.angular.z * 0.3))/(2*0.1)
        print(self.Vr, self.Vl)
    
    def wheel_vels(self):
        while not rospy.is_shutdown():
            self.convert_vel()
            self.pub1.publish(self.Vr)
            self.pub2.publish(self.Vl)
            rospy.sleep(0.1)

if __name__=='__main__':
    rospy.init_node('convert_velo')
    x = read_convert()
    x.wheel_vels()