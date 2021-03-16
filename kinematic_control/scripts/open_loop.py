import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
from tf.transformations import euler_from_quaternion

rospy.init_node('kinematic_controller', anonymous=True)

class VelocityController():
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        rospy.sleep(0.1)

    def move(self, linear_velocity, angular_velocity):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel.publish(msg)

class OdometryReader():
    def __init__(self, topic):
        self.pose = {}
        self.trajectory = []
        self.topic = topic
        self.subscribe()

    def callback(self, msg):
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.pose['x'], self.pose['y']))
        (_, _, self.pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                            msg.pose.pose.orientation.y, 
                                                            msg.pose.pose.orientation.z, 
                                                            msg.pose.pose.orientation.w])
    def subscribe(self):
        self.subscriber = rospy.Subscriber(self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self):
        np.save('trajectory',self.trajectory)
        self.subscriber.unregister()

from utilities import set_position
set_position(0,0,0)
velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')
rospy.sleep(1)

velocity.move(1,-1)
rospy.sleep(2*pi/4)
velocity.move(1,1)
rospy.sleep(2*pi/4)
velocity.move(1,0)
rospy.sleep(1)
velocity.move(1,1)
rospy.sleep(2*pi/4)
velocity.move(1,2)
rospy.sleep(2*pi*0.5/8)
velocity.move(1,-2)
rospy.sleep(2*pi*0.5/8)
velocity.move(1,0)
rospy.sleep(1)
velocity.move(1,2)
rospy.sleep(2*pi*0.5/2)
velocity.move(1,-1)
rospy.sleep(2*pi/4)
velocity.move(1,0)
rospy.sleep(1.5)

velocity.move(0,0)
odometry.unregister()
error = math.hypot(odometry.pose['x'], odometry.pose['y'])
print('Final positioning error is %.2fm' % error)