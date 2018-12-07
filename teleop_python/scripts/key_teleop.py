#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import click

def talker():
    #Init
    pub_name = "/cmd_vel"
    twist = Twist()
    velocity = 2

    #Publishing velocity cmd
    pub = rospy.Publisher(pub_name, Twist)

    #Starts the node
    rospy.init_node('Key_teleop')
    
    rate = rospy.Rate(10)
    
    rospy.loginfo("Press z,q,s or d to move :")
    while not rospy.is_shutdown():
        key = click.getchar()
        #print(key)
        if key == 'z':
            twist.linear.x = velocity
            twist.angular.z = 0
        elif key == 's':
            twist.linear.x = -velocity
            twist.angular.z = 0

        if key == 'q':
            twist.linear.x = 0
            twist.angular.z = velocity
        elif key == 'd':
            twist.linear.x = 0
            twist.angular.z = -velocity

        pub.publish(twist)
        rate.sleep()


if __name__ == '__main__':
    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
