#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

"""
1p : teleop
1p : slam
5p : path finding
4p : suivi de chemin
1p : repartition travail
1p : qualité video
2p : contenu video
1p : qualité prez
4p : contenu prez
bonus : evitement obstacle, arret automatique
"""

twist = Twist()

def talker():

    #Publishing velocity cmd
    pub = rospy.Publisher('turtle1/cmd_vel', Twist)

    #Starts the node
    rospy.init_node('Key_teleop')

    rate = rospy.Rate(10)

    twist.linear.x = 4*data.axes[1]
    twist.angular.z = 4*data.axes[0]
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()


if __name__ == '__main__':
    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
