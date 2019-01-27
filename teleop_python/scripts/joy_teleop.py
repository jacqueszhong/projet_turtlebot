#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time


pub_name = '/cmd_vel'
twist = Twist()

speed = 0.3
speedw = 0.6
a_push = False
flag = 0

#Callback pour le suscriber au node joystick
def callback(data):
    global twist
    global a_push
    global flag

    coef = 1+3*(1-data.axes[5])/2 

    twist.linear.x = coef*speed*data.axes[1] 
    twist.angular.z = coef*speedw*data.axes[0]

    #seuils activation
    if abs(data.axes[1]) <= 0.125:
        twist.linear.x = 0
    
    if abs(data.axes[0]) <= 0.125:
        twist.angular.z = 0  


    if data.buttons[0]:
        a_push = True
    else :
        if a_push == True :
            flag=1
            a_push = False




def talker():
    global flag
    pub = rospy.Publisher(pub_name, Twist)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy_teleop')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo(twist)

        if a_push :
            print("Joy control activated")
            pub.publish(twist)
        elif flag :
            flag=0
            twist.linear.x = 0
            twist.angular.z = 0
            pub.publish(twist)

        rate.sleep()

if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
