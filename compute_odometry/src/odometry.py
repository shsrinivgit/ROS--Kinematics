#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from simple_robot_gazebo.msg import encoders
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
import time
from math import pi, sin, cos
import tf
from tf.broadcaster import TransformBroadcaster
class odom():
    def __init__(self):
        self.sub = rospy.Subscriber('/encoders', encoders, self.my_callback)
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.odo = Odometry()
        self.read = encoders()
        self.Fistleftenco_tick = 0
        self.Firstrightenco_tick = 0
        rospy.sleep(0.5)
        self.R = 0.1
        self.L = 0.3
        self.N = 360
        self.x = 0
        self.y = 0
        self.odom_broadcaster = TransformBroadcaster()
        self.theta = 0.0
        self.last_time = rospy.Time.now()

    def my_callback(self,msg):
        self.read = msg
        self.Lastleftenco_tick = self.read.encoderTicks[0]
        self.Lastrightenco_tick = self.read.encoderTicks[1]
    
    def readings(self):
        while not rospy.is_shutdown():
            present_time = rospy.Time.now()
            Dr = 2*pi*self.R*(self.Fistleftenco_tick - self.Lastleftenco_tick)/self.N
            Dl = 2*pi*self.R*(self.Firstrightenco_tick - self.Lastrightenco_tick)/self.N

            self.Firstrightenco_tick = self.Lastrightenco_tick
            self.Fistleftenco_tick = self.Lastleftenco_tick
            
            v = (Dr + Dl)/2
            w = (Dr-Dl)/self.L

            self.x += v*cos(self.theta)
            self.y += v*sin(self.theta)
            self.theta += w

            #get the quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

            self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            present_time,
            "link_chassis",
            "odom"
            )

            self.odo.header.stamp = present_time
            self.odo.header.frame_id = "odom"

            self.odo.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

            self.odo.child_frame_id = "link_chassis"
            self.odo.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))

            self.pub.publish(self.odo)

            self.last_time = present_time
            rospy.sleep(0.1)
if __name__=='__main__':
    rospy.init_node('odom_reader')
    x = odom()
    x.readings()