#! /usr/bin/env python

from nav_msgs.msg import Odometry
import numpy as np
import rospy
from math import cos, sin, radians
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

rospy.init_node('abs_motion')

def odom_callback(msg):
    global phi   
    position = msg.pose.pose.position
    (_, _, phi) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                         msg.pose.pose.orientation.y, 
                                         msg.pose.pose.orientation.z, 
                                         msg.pose.pose.orientation.w])
    
position_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
pub = rospy.Publisher('/wheel_speed',Float32MultiArray, queue_size=5)
rospy.sleep(0.5)

def velocity2twist(dphi,dx, dy):
    global phi
    mat1 = np.array([[1,0,0],[0,cos(phi),sin(phi)],[0,-sin(phi),cos(phi)]])
    mat2 = np.transpose(np.array([dphi,dx,dy]))
    wx,vx,vy = np.dot(mat1,mat2)
    return wx, vx, vy

def twist2wheels(W_bz, V_bx, V_by):
    l = 0.250
    w = 0.274
    r = 0.127
    mat1 = np.matrix([[-l-w,1,-1],[l+w,1,1],[l+w,1,-1],[-l-w,1,1]])
    mat2 = np.transpose(np.matrix([W_bz, V_bx, V_by]))
    u = (1/r)*np.dot(mat1,mat2)
    return u

dx = 0;dy = 1
for i in range(2,6):
    for _ in range(100):
        wz, vx, vy = velocity2twist(dphi=1,dx=dx, dy=dy)
        u = twist2wheels(wz, vx, vy)
        msg = Float32MultiArray(data=u)
        pub.publish(msg)
        rospy.sleep(0.03)
    dx = -1 +i%2;dy = -1 + (i+1)%2
    if i==4:
        dx = 1;dy = 0
    print(dx,dy)

stop = [0,0,0,0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)