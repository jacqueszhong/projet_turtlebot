# coding: utf-8

#Interpolation du path publié
#Calcul point le plus proche, calcul erreur vecteur vitesse (avec bouclage)

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Joy
import tf

import time
import math


class Controller:

	def __init__(self):

		self._cmd = Twist()

		self._speed = 0.3

		self._start_point = Point()
		self._path = 0
		self._count = 0
		
		self._joy_run = False
		self._stop = False

		rospy.init_node("Controller")

		#Subscribers
		self.tf_listener = tf.TransformListener()
		rospy.Subscriber("path",Path,self.path_callback)
		rospy.Subscriber("joy",Joy, self.joy_callback)

		#Publishers 
		self.pub_cmd = rospy.Publisher("/cmd_vel",Twist,queue_size=5)
		self.pub_target = rospy.Publisher("/target_point",Point,queue_size=1)


	def joy_callback(self,data):
		#print("GOT_JOY : "+str(data))

		"""
		if data.buttons[0] :
			self._joy_run = True
		else :
			self._joy_run = False

		"""

		self._joy_run = data.buttons[0]

		if data.buttons[2] == 1 :
			
			(trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
			self._start_point.x = trans[0]
			self._start_point.y = trans[1]

			print("SETTING INITIAL POINT FROM JOYSTICK : "+str(self._start_point))

		if data.buttons[3] == 1 :
			self.pub_target.publish(self._start_point)
			self._stop = True

			print("GETTING BACK HOME : "+str(self._start_point))



	def path_callback(self,data):
		print("GOT PATH : "+str(data))
		self._stop = False
		self._path = data
		self._count = 0

	def get_robot_angle(self, quaternion):
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
		return yaw

	def get_robot_pos(self):
		(trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
		return (trans,rot)

	def get_distance(self,p1,p2):
		d = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
		return d

	def get_vector_angle(self,p1,p2):
		vec = (p2[0]-p1[0], p2[1]-p1[1])
		return math.atan2(vec[1],vec[0])


	def get_target_pos(self):
		"""
		Find next path point
		"""

		if not self._path :
			print("No path to follow !")
			return False

		#End of path
		if self._count >= len(self._path.poses) :
			print("End of path ! ")
			self._path = 0
			self._count = 0
			return False

		point = self._path.poses[self._count].pose.position
		return (point.x,point.y)

	def follow_path(self):
		"""
		Follow path stored in self._path
		"""

		#Retrieve some information
		target_pos = self.get_target_pos()


		if target_pos == False: #do nothing if no target
			self._cmd.angular.z = 0
			self._cmd.linear.x = 0

		else : 
			(rob_pos,rob_rot) = self.get_robot_pos()

			distance = self.get_distance(rob_pos,target_pos)
			print("ROB="+str(rob_pos))
			print("TARGET="+str(target_pos))
			print("DISTANCE="+str(distance))

			if distance <= 0.5:
				print("FOUND TARGET POINT")
				self._count += 1


			#Compute angle error
			theta_rob = self.get_robot_angle(rob_rot)
			theta_target = self.get_vector_angle(rob_pos,target_pos)
			err = theta_target - theta_rob

			if err > math.pi:
				err -= 2*math.pi
			if err < -math.pi:
				err += 2*math.pi
			

			print("robA="+str(theta_rob))
			print("targetA="+str(theta_target))

			#Compute command
			Kp = 1
			Kcor = 0.2
			self._cmd.angular.z = Kp * err
			correction = abs(Kcor * self._cmd.angular.z)
			if correction < 0:
				correction = 0

			self._cmd.linear.x = self._speed - correction



	def avoid_obstacle(self):
		print("AVOIDING")
		return Twist()

	def detect_obstacle(self):
		return False

	def explore(self):
		return Twist()

	def run(self):

		# Do nothing
		if self._stop :
			print("stop")
			self._cmd.linear.x = 0
			self._cmd.angular.z = 0

		#Action selection
		if self._joy_run : #Let joystick control
			print("joy_run")

			return

		elif self.detect_obstacle():
			print("avoid")
			self.avoid_obstacle()

		elif self._path:
			print("path")
			self.follow_path()

		else :
			print("explore")
			self.explore()

		#Send command
		#print("x="+str(self._cmd.linear.x)+",z="+str(self._cmd.angular.z))
		self.pub_cmd.publish(self._cmd)





if __name__ == '__main__':
	print("Controller_main")

	controller = Controller()

	while not rospy.is_shutdown():
		controller.run()

		time.sleep(0.5)
