#!/usr/bin/env python
import rospy
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import tf

from make_graph import *


class Planner :


	def __init__(self):
		rospy.init_node("Planner")

		#Subscribers
		self.tf_listener = tf.TransformListener()
		rospy.Subscriber("map",OccupancyGrid,self.map_callback)
		rospy.Subscriber("map_metadata",MapMetaData,self.meta_map_callback)

		#Publishers 
		self.pub_path = rospy.Publisher("/path",Path)


	def map_callback(self,data):
		print("Map : "+str("data"))

	def meta_map_callback(self,data):
		print("Meta map : "+str("data"))
		self.map_resolution = data.resolution
		self.map_origin = data.origin
		self.map_width = data.map_width
		self.map_height = data.map_height
		

	def get_target(self):
		return 0

	def get_map_info(self):
		return 0

	def find_path(self):
		"""
		rrt = BuildRRT()
		rrt.init( (200,300), initPos = (260,360) )
		rrt.load_pgm_map("test_map.pgm")

		g = rrt.runRRT()

		return g.get_path_pos()

		"""
		#A COMPLETER QUAND RRT MARCHE



		return Path() 


	def run_planification(self):

		#Get info on robot position
		try :
			(trans,rot) = self.tf_listener.lookupTransform('/base_link','/map',rospy.Time(0))
			#print("Trans = "+str(trans))
			#print("Rot = "+str(rot))  #quaternion

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			#print("Not valid tf lookupTransform")
			pass

		#Retrieve path
		path = self.find_path()
		self.pub_path.publish(path)






if __name__ == '__main__':
	print("Planner_node main")

	planner = Planner()

	while not rospy.is_shutdown():
		planner.run_planification()		


		#path = find_path()



