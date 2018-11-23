#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


pub_name = '/cmd_vel'
twist = Twist()

#Callback pour le suscriber au node joystick
def callback(data):
    global twist

    twist.linear.x = 4*data.axes[1] 
    twist.angular.z = 4*data.axes[0]  

    #seuils activation
    if abs(data.axes[1]) <= 0.125:
        twist.linear.x = 0
    
    if abs(data.axes[0]) <= 0.125:
        twist.angular.z = 0  
    


def talker():
    pub = rospy.Publisher(pub_name, Twist)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy_teleop')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
