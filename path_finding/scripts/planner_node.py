#!/usr/bin/env python
import rospy
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose,PoseStamped
import tf
import time

#from make_graph import *
from rrt import *
from image_processing import *

MANUAL = 0
DEBUG = 1
VERBOSE = 0
MAP_NAME = "test_map_buvoir.pgm"

def pathPoses_building(path):
    pathP=Path()
    pathP.header.frame_id = 'map'
    for pos in path:
        loc = PoseStamped()#moved in to for loop
        loc.pose.position.x = pos[0]
        loc.pose.position.y = -pos[1]
        loc.pose.position.z = 0        
        pathP.poses.append(loc)
    return pathP

class Planner :

	def __init__(self):
		self._grid = 0
		self._map = np.array([])
		self._map_resolution = 0
		self._map_origin = Pose()
		self._map_width = 0
		self._map_height = 0
		self._posOffset = (0,0)

		rospy.init_node("Planner")

		#Subscribers
		self.tf_listener = tf.TransformListener()
		rospy.Subscriber("map",OccupancyGrid,self.map_callback)
		#rospy.Subscriber("map_metadata",MapMetaData,self.meta_map_callback)

		#Publishers 
		self.pub_path = rospy.Publisher("/path",Path,queue_size=5)


	def map_callback(self,data):
		self._grid = data.data;
		self._map_resolution = data.info.resolution
		self._map_origin = data.info.origin
		self._map_width = data.info.width
		self._map_height = data.info.height

		if VERBOSE :
			print("Map retrieved : width={0},height={1},res={2},origin={3}".format(self._map_width,self._map_height,self._map_resolution,self._map_origin))

	def meta_map_callback(self,data):
		print("Meta map : "+str("data"))
		self._map_resolution = data.resolution
		self._map_origin = data.origin
		self._map_width = data.map_width
		self._map_height = data.map_height
		

	def get_target(self):
		return 0

	def get_map_info(self):
		return 0

	def find_path(self, img, initPos, targetPos):
		
		#load initial and target position
		if not MANUAL:

			cv2.imshow("img",img)
			cv2.waitKey(1000)

			#img = cv2.imread(MAP_NAME,1)
			#img_inv= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
			ret,img_inv = cv2.threshold(img,127,255,cv2.THRESH_BINARY_INV)

			rrt = RRT(img_inv)
			rrt._initPos = initPos
			rrt._targetPos = targetPos
			rrt.start_connect(img)

		#load Position from subscriber
		else:
			img_map = cv2.imread(MAP_NAME,1)
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
		print("PLANNER_NODE : path in pix : "+str(path))
		return path

	def convert_path_real(self,tab_path):
		res = []

		for p in tab_path:
			newx = (p[0] + self._posOffset[0])*self._map_resolution + self._map_origin.position.x
			newy = (p[1] + self._posOffset[1])*self._map_resolution + self._map_origin.position.y
			res.append((newx,newy))
		return res

	def convert_to_pixel(self,coord):
		res = [0,0,0]

		res[0] = ((res[0] - self._map_origin.position.x) / self._map_resolution) - self._posOffset[0]
		res[1] = ((res[1] - self._map_origin.position.y) / self._map_resolution) - self._posOffset[1]
		res[2] = coord[2]

		return res

	def run_map_process(self):
		#Process image data from occupancy grid

		#Process OccupencyGrid msg
		if not self._grid :
			print("No grid sorry.")
			return

		if DEBUG : #Retrieve map from fixed location
			m=cv.imread('../mymap.pgm',0) #Debug map
		else : #Retrieve map from topic
			m =  grid_process(self._grid,self._map_width)
			ret,m = cv2.threshold(m,127,255,cv2.THRESH_BINARY_INV)

		res = image_process(m)

		self._posOffset = res[0]
		self._map = res[1]
		#cv.imshow("processed map",self._map)

		if (not self._map_width) or VERBOSE:
			print("posOffset="+str(self._posOffset))
			print("res="+str(self._map_resolution))
			print("origin="+str(self._map_origin))
			print("height="+str(self._map_height))
			print("width="+str(self._map_width))


	def run_planification(self):

		#Init some parameters
		self._initPos = (100,100)
		self._targetPos = (250,140)
		if self._map.size == 0 :
			print("No map sorry.")
			return

		#Get info on robot position
		try :
			(trans,rot) = self.tf_listener.lookupTransform('/base_link','/map',rospy.Time(0))
			print("Trans = "+str(trans))
			print("Rot = "+str(rot))  #quaternion
			pos = self.convert_to_pixel(trans)
			print("pos:"+str(pos))
			self._initPos = (int(pos[0]), int(pos[1]))
			print("initPos:"+str(self._initPos))

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("Not valid tf lookupTransform")
			pass

		#Find a path in pixel
		path = self.find_path(self._map,self._initPos,self._targetPos)

		#Convert path in meters
		path = self.convert_path_real(path)
		print("\t path in meters : "+str(path))


		
		#Publish ROS path
		path_msg = pathPoses_building(path)
		self.pub_path.publish(path_msg)


	#def show_path(self, map, path):




if __name__ == '__main__':
	print("Planner_node main")

	planner = Planner()

	while not rospy.is_shutdown():
		planner.run_map_process()
		planner.run_planification()	




