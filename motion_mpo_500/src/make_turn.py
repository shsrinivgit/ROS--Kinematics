#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
rospy.init_node('make_turnn')

def twist2wheels(W_bz, V_bx, V_by):
    l = 0.250
    w = 0.274
    r = 0.127
    mat1 = np.matrix([[-l-w,1,-1],[l+w,1,1],[l+w,1,-1],[-l-w,1,1]])
    mat2 = np.transpose(np.matrix([W_bz, V_bx, V_by]))
    print(mat1,mat2)
    u = (1/r)*np.dot(mat1,mat2)
    print(u)
    return u

pub = rospy.Publisher('/wheel_speed', Float32MultiArray, queue_size=1)
rospy.sleep(0.1)
u = twist2wheels(1.5, 1, 0)
msg = Float32MultiArray(data=u)
pub.publish(msg)
rospy.sleep(1)
stop = [0,0,0,0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)