#!/usr/bin/env python
import rospy
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import tf

#from make_graph import *
from rrt import *

GUI = 1

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

	def find_path(self, img, initPos, targetPos):

		
		#load initial and target position
		if not GUI:
			rrt = RRT(img)
			rrt._initPos = init_pos
			rrt._targetPos = target_pos
			rrt.start_connect()

		#load Position from subscriber
		else:
			img_map = cv2.imread("test_map.pgm",1)
			img_map2= cv2.cvtColor(img_map,cv2.COLOR_BGR2GRAY)
			ret,img_map2 = cv2.threshold(img_map2,127,255,cv2.THRESH_BINARY_INV)

			# Create rrt class support
			rrt = RRT(img_map=img_map2)

			cv2.imshow('RRT', img_map)
			cv2.setMouseCallback('RRT',rrt.pos_define, param=img_map)
			while(rrt.CLICK_COUNTER<2):
				cv2.imshow('RRT', img_map)
				if cv2.waitKey(20) & 0xFF == 27:
					break
			print("ok")
			rrt.start_connect(img_map)
			#rrt.interpolation_path()	

			while(1):
				cv2.imshow('RRT', img_map)
				if cv2.waitKey(10) & 0xFF == 27:
					break
			cv2.destroyAllWindows()

		
		path = rrt._path
		print("PLANNER_NODE : quit with path = "+str(path))
		return path


	def run_planification(self):

		#Get info on robot position
		try :
			(trans,rot) = self.tf_listener.lookupTransform('/base_link','/map',rospy.Time(0))
			#print("Trans = "+str(trans))
			#print("Rot = "+str(rot))  #quaternion

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			#print("Not valid tf lookupTransform")
			pass

		#Get path
		img_map = cv2.imread("test_map.pgm",1)
		img_map2= cv2.cvtColor(img_map,cv2.COLOR_BGR2GRAY)
		ret,img_map2 = cv2.threshold(img_map2,127,255,cv2.THRESH_BINARY_INV)

		self._img = img_map2
		self._initPos = (160,100)
		self._targetPos = (250,170)

		path = self.find_path(self._img,self._initPos,self._targetPos)
		print(path)
		self.pub_path.publish(path)






if __name__ == '__main__':
	print("Planner_node main")

	planner = Planner()

	while not rospy.is_shutdown():
		planner.run_planification()		


		#path = find_path()



