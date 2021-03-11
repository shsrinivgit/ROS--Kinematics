import math, rospy
from utilities import set_model_state, get_model_state, \
                      pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_about_axis
from tf.transformations import quaternion_multiply

#for postion the table
position = Point(1,0,0)
q = quaternion_about_axis(math.radians(90), (0,0,1))
orientation = Quaternion(*q)
set_model_state('table', Pose(position, orientation))
rospy.sleep(0.1)

#for position the coke cans
pause_physics()
x, y, z = 0.4, 0, 1.05
for angle in range(0,360,30):
    i = angle/30
    print(i)
    theta = math.radians(angle)
    q = quaternion_about_axis(math.radians(90), (0,1,0))
    # orientation = Quaternion(*q)
    q_z = quaternion_about_axis(theta, (0,0,1))
    q_yz = quaternion_multiply(q_z, q)
    orientation = Quaternion(*q_yz)
    xp = 0.4 * math.cos(theta)+1 #equation of the circle
    yp = 0.4 * math.sin(theta)
    position = Point(xp, yp, z)
    set_model_state('coke_can_'+str(i), Pose(position, orientation))
    rospy.sleep(0.1)
