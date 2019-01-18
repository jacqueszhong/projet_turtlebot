# coding: utf-8

#Interpolation du path publié
#Calcul point le plus proche, calcul erreur vecteur vitesse (avec bouclage)

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Joy, LaserScan
import tf

import time
import math


class Controller:

	def __init__(self):

		self._cmd = Twist()

		self._speed = 0.3
		self._speedw = 0.6

		self._start_point = Point()
		self._path = 0
		self._count = 0
		
		self._joy_run = False
		self._joy_data = Joy()

		self._stop = False

		self._min_val = 999999

		rospy.init_node("Controller")

		#Subscribers
		self.tf_listener = tf.TransformListener()
		rospy.Subscriber("path",Path,self.path_callback)
		rospy.Subscriber("joy",Joy, self.joy_callback)
		rospy.Subscriber("scan",LaserScan, self.laser_callback)

		#Publishers 
		self.pub_cmd = rospy.Publisher("/cmd_vel",Twist,queue_size=5)
		self.pub_target = rospy.Publisher("/target_point",Point,queue_size=1)

	def laser_callback(self,data):

		#Find desired ranges
		min_angle = -math.pi/3
		max_angle = math.pi/3
		inc_min = int((min_angle - data.angle_min) / data.angle_increment)
		inc_max = int((data.angle_max - max_angle) / data.angle_increment)
		
		#print("inc_min="+str(inc_min)+", inc_max="+str(inc_max))
		
		if inc_min < 0:
			inc_min = 0
		if inc_max < 0:
			inc_max = 0


		#print("inc_min="+str(inc_min)+", inc_max="+str(inc_max))
		self._min_val = min(data.ranges[inc_min : len(data.ranges)-inc_max])
		min_ind = data.ranges.index(self._min_val)

		self._obs_angle = data.angle_min + min_ind * data.angle_increment
		#print("min_val="+str(self._min_val)+",obs_angle="+str(self._obs_angle))



	def joy_callback(self,data):

		#(A)
		"""
		if data.buttons[0]:
			self._joy_run = True
			self._joy_data = data
		else :
			if self._joy_run : #Transition from pushed to unpushed
				cmd = Twist()
				self.pub_cmd.publish(cmd)
			self._joy_run = False
		"""
		self._joy_run = data.buttons[0]

		#(X)
		if data.buttons[2] == 1 :
			(trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
			self._start_point.x = trans[0]
			self._start_point.y = trans[1]

			print("SETTING INITIAL POINT FROM JOYSTICK : "+str(self._start_point))

		#(Y)
		if data.buttons[3] == 1 :
			self.pub_target.publish(self._start_point)
			self._stop = True

			print("GETTING BACK HOME : "+str(self._start_point))


		#(RT)
		if data.axes[5] != 1:
			self._coef_vel = 1+3*(1-data.axes[5])/2 



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
			Kp = 0.5
			Kcor = 0.1
			self._cmd.angular.z = self._coef_vel*Kp * err

			correction = abs(Kcor * self._cmd.angular.z)
			if correction < 0:
				correction = 0
			self._cmd.linear.x = self._coef_vel*(self._speed - correction)
			if self._cmd.linear.x < 0:
				self._cmd.linear.x = 0


	def avoid_obstacle(self):
		print("AVOIDING")

		Kp = 2
		target = math.pi/2


		print("obs="+str(self._obs_angle))

		if self._obs_angle > 0 : #Obstacle gauche
			err = (target - self._obs_angle)

		else : #Obstacle droite
			err = (-target - self._obs_angle)

		self._cmd.angular.z = -Kp * err

		#self._cmd.linear.x = self._speed * (target - abs(err))**3
		self._cmd.linear.x = 0


	def detect_obstacle(self):
		obs_thresh = 0.5

		return self._min_val < obs_thresh


	def explore(self):
		self._cmd.linear.x = 0
		self._cmd.angular.z = 0

	def joy_control(self):

		#Velocity intensity
		coef = 1+(self._joy_data.axes[5]+1)/2 


		#Velocity direction
		self._cmd.linear.x = coef*self._speed*self._joy_data.axes[1]
		self._cmd.angular.z = coef*self._speedw*self._joy_data.axes[0] 

		#seuils activation
		if abs(self._joy_data.axes[1]) <= 0.125:
			self._cmd.linear.x = 0

		if abs(self._joy_data.axes[0]) <= 0.125:
			self._cmd.angular.z = 0  

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
			#self.joy_control()

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
		print("CMD : x="+str(self._cmd.linear.x)+",z="+str(self._cmd.angular.z))
		self.pub_cmd.publish(self._cmd)





if __name__ == '__main__':
	print("Controller_main")

	controller = Controller()
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		controller.run()
		rate.sleep()
