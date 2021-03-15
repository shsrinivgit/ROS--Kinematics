import rospy, math, numpy as np
from std_msgs.msg import Float32MultiArray
from utilities import reset_world

rospy.init_node('holonomic_controller', anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
rospy.sleep(0.1)\

def motion(direction):
    msg = Float32MultiArray(data=direction)
    pub.publish(msg)
    rospy.sleep(1)


backward = [-1, -1, -1, -1]
motion(backward)
left = [-1, 1, -1, 1]
motion(left)
right = [1, -1, 1, -1]
motion(right)
clk_rot = [-1, 1, 1, -1]
motion(clk_rot)
antclk_rot = [1, -1, -1, 1]
motion(antclk_rot)
stop = [0,0,0,0]
motion(stop)


# msg = Float32MultiArray(data=left)
# pub.publish(msg)
# rospy.sleep(1)

# right = [-1, 1, -1, 1]
# msg = Float32MultiArray(data=right)
# pub.publish(msg)
# rospy.sleep(1)

# clk_rot = [-1, 1, -1, 1]
# msg = Float32MultiArray(data=clk_rot)
# pub.publish(msg)
# rospy.sleep(1)

