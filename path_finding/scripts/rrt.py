import sys
import random
import time

import numpy as np
from math import hypot, sqrt
from random import randrange as rand
from numpy import linalg
import cv2

import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

from collections import defaultdict
import math


GUI = 1  # activate graphic interface
#CLICK_COUNTER = 0  # count number of right click on the interface
DELTA_RADIUS = 10 #Allowed error to reach target
MAX_DIST = 20 #Incremental distance

# - Function to calculate distance between 2 points:
def dist(p1,p2):
  return hypot(p2[0]-p1[0],p2[1]-p1[1])

# - Vertex class:
class Vertex:
  def __init__(self,pos,parent):
    self.pos = pos
    self.parent = parent

# - RRT class :
class RRT :

	CLICK_COUNTER = 0


	def __init__(self,img_map):
		print("Initializing RRT")
		self._pgm_format = ''
		self._initPos = (160,80)
		self._targetPos = (90,180)
		self._map = img_map.copy()
		self._map[self._map==255] = 1
		self._height = img_map.shape[0]
		self._width = img_map.shape[1]
		self._path=[]

	# - Load the current position of the robot as the initial one and the target position from topics ROS
	def loadPos(self): #TODO
	# charge pos of the robot int the map from image_processing
	# charge final pos of the robot from rviz
		return 


	def start(self,img_map=None):
		"""
		RRT based algorithm
		"""

		goal = False
		self._path=[]
		newvertex = Vertex(self._initPos,None)
		vertices = [newvertex]
		lin_dist = dist(self._initPos,self._targetPos)

		# main loop
		while not goal:
			# create random point
			newpoint = (rand(self._width),rand(self._height))
			nearest_dist = float('inf')
			# look for the nearest point in the tree
			for v in vertices:
				currdist = dist(newpoint,v.pos)
				if currdist < nearest_dist:
					nearest = v
					nearest_dist = currdist
			# take into account the non-holonomy of the robot
			newpoint=self.steer(nearest.pos,newpoint)

			# try to connect the point to the tree
			if not self.collide_line(nearest.pos,newpoint):
				newvertex = Vertex(newpoint,nearest)
				vertices.append(newvertex)
				
				if GUI:
					img_map=cv2.circle(img_map,newpoint,2,(255,0,0),-1)
					img_map=cv2.line(img_map,newpoint,nearest.pos,(255,0,0),1)
					cv2.imshow('RRT', img_map)
					cv2.waitKey(20)

				# test if the goal is reached
				if self.test_goal(newpoint):
					goal=True

		# build the path
		self._path =[]
		currvertex = newvertex
		while currvertex.parent:
			self._path.append(currvertex.pos)
			currvertex = currvertex.parent
		self._path.append(currvertex.pos)
		self._path.reverse()

		self.shorten_path()

		if GUI:
			self.draw_path(img_map)

	def start_connect(self,img_map=None):
		"""
		RRT-connect based algorithm
		"""

		goal = False
		count = 1
		self._path=[]
		newvertex=[]
		vertices=[]
		newvertex.append(Vertex(self._initPos,None))
		newvertex.append(Vertex(self._targetPos,None))
		vertices.append([newvertex[0]])
		vertices.append([newvertex[1]])

		# main loop
		while not goal:

			count = (count + 1) % 2 # to select whch tree we are working with, from the initial point (0) or the final one (1)
			# create random point
			newpoint = (rand(self._width),rand(self._height))
			nearest_dist = float('inf')
			# look for the nearest point in the tree
			for v in vertices[count]:
				currdist = dist(newpoint,v.pos)
				if currdist < nearest_dist:
					nearest = v
					nearest_dist = currdist
			# take into account the non-holonomy of the robot
			#newpoint=self.steer(nearest.pos,newpoint)

			# try to connect the point to the tree
			if not self.collide_line(nearest.pos,newpoint):
				newvertex[count] = Vertex(newpoint,nearest)
				vertices[count].append(newvertex[count])

				if GUI:
					if count % 2 == 0:
						img_map=cv2.circle(img_map,newpoint,2,(255,0,0),-1)
						img_map=cv2.line(img_map,newpoint,nearest.pos,(255,0,0),1)
					else:
						img_map=cv2.circle(img_map,newpoint,2,(255,125,0),-1)
						img_map=cv2.line(img_map,newpoint,nearest.pos,(255,125,0),1)
					cv2.imshow('RRT', img_map)
					cv2.waitKey(10)

				# try to connect the point to the other tree
				nearest_dist = float('inf')
				for v in vertices[(count+1)%2]:
					currdist = dist(newpoint,v.pos)
					if currdist < nearest_dist:
						nearest = v
						nearest_dist = currdist

				# take into account the non-holonomy of the robot
				newpoint=self.steer(nearest.pos,newpoint)

				# try to connect the point to the tree
				if not self.collide_line(newpoint,nearest.pos):
					# test if the goal is reached
					if newpoint == vertices[count][-1].pos:
						goal = True
					else:
						newvertex[(count+1)%2] = Vertex(newpoint,nearest)
						vertices[(count+1)%2].append(newvertex[(count+1)%2])
						if GUI:
							img_map=cv2.circle(img_map,newpoint,2,(255,120,0),-1)
							img_map=cv2.line(img_map,newpoint,nearest.pos,(255,120,0),1)
							cv2.imshow('RRT', img_map)
							cv2.waitKey(10)

		# build the path
		self._path =[]

		# building depends on which tree finished the algorithm
		if count == 0:
			currvertex1 = newvertex[count]
			currvertex2 = nearest
		else:
			currvertex1 = nearest
			currvertex2 = newvertex[count]

		while currvertex1.parent:
			self._path.append(currvertex1.pos)
			currvertex1 = currvertex1.parent
		self._path.append(currvertex1.pos)
		self._path.reverse()
		while currvertex2.parent:
			self._path.append(currvertex2.pos)
			currvertex2 = currvertex2.parent
		self._path.append(currvertex2.pos)

		self.shorten_path()

		if GUI:
			self.draw_path(img_map)

	def shorten_path(self):
		"""
		Shorten the saved path with minimum straight lines
		"""
		path=self._path[:]
		self._path=[]
		self._path.append(path[0])
		i = 0
		j = i + 2
		while j < len(path):
			if self.collide_line(path[i],path[j]):
				self._path.append(path[j-1])
				i = j-1
			j += 1
		self._path.append(path[j-1])

	"""def interpolation_path(self):
		
		#Change the path to make it accessible to the robot
		
		x = [coord[0] for coord in self._path]
		y = [self._height - coord[1] for coord in self._path]
		print(x)
		print(y)
		f = interp1d(x, y)
		f2 = interp1d(x, y, kind='cubic')


		xnew = np.arange(x[0], x[-1], 2)
		plt.plot(x, y, 'o', xnew, f(xnew), '-', xnew, f2(xnew), '--')
		plt.legend(['data', 'linear', 'cubic'], loc='best')
		plt.show()"""


	def draw_path(self,img_map):
		"""
		Draw the path in red into the img
		"""
		for i in range(1,len(self._path)):
			img_map=cv2.line(img_map,self._path[i-1],self._path[i],(0,0,255),4)
			cv2.imshow('RRT', img_map)
			cv2.waitKey(10)


	def steer(self,startpoint,dirpoint): #TODO prend en compte les contraintes du robot
		"""
		Take into account the non_holonomy of the robot
		"""
		if dist(startpoint,dirpoint)<MAX_DIST:
			return dirpoint
		else:
			d = sqrt((dirpoint[1]-startpoint[1])*(dirpoint[1]-startpoint[1])+(dirpoint[0]-startpoint[0])*(dirpoint[0]-startpoint[0]))
			return (int(startpoint[0]+MAX_DIST/d*(dirpoint[0]-startpoint[0])),int(startpoint[1]+MAX_DIST/d*(dirpoint[1]-startpoint[1])))


	def collide_line(self,start,end):
		"""
		Test if two point are separated by an obstacle by tracing a line between them
		"""
		img = np.zeros((self._height,self._width))
		img = cv2.line(img,start,end,1,1)
		intersection = np.logical_and( self._map, img)
		if np.count_nonzero(intersection)==0:  # No collision
			return False
		else:  #Collision
			return True

	def collide_circle(self,point,radius):
		"""
		Test if a point if circle collide with an obstacle
		"""
		img = np.zeros((self._height,self._width))
		img = cv2.circle(img,point,radius,1,-1)
		intersection = np.logical_and( self._map, img)
		if np.count_nonzero(intersection)==0:  # No collision
			return False
		else:  #Collision
			return True


	def test_goal(self,point):
		"""
		Test if the target point has been reached regarding a certain error
		"""
		if (self._targetPos[0]-DELTA_RADIUS<point[0]<self._targetPos[0]+DELTA_RADIUS) and (self._targetPos[1]-DELTA_RADIUS<point[1]<self._targetPos[1]+DELTA_RADIUS):
			return True
		else:
			return False

	# mouse callback function
	def pos_define(self,event,x,y,flags,param):
		#global CLICK_COUNTER
		if event == cv2.EVENT_LBUTTONDBLCLK:
			if not self.collide_circle((x,y),DELTA_RADIUS):
				if self.CLICK_COUNTER==0:
					self._initPos = (x,y)
					print("InitPos = ("+str(x)+","+str(y)+")")
					param=cv2.circle(param,(x,y),DELTA_RADIUS,(0,255,0),-1)
				if self.CLICK_COUNTER==1:
					self._targetPos = (x,y)
					print("targetPos = ("+str(x)+","+str(y)+")")
					param=cv2.circle(param,(x,y),DELTA_RADIUS,(0,0,255),-1)
					#param = cv2.line(param,self._initPos,self._targetPos,0,1)
				self.CLICK_COUNTER +=1


if __name__ == '__main__':
	img_map = cv2.imread("test_map.pgm",1)
	img_map2= cv2.cvtColor(img_map,cv2.COLOR_BGR2GRAY)
	ret,img_map2 = cv2.threshold(img_map2,127,255,cv2.THRESH_BINARY_INV)

	# Create rrt class support
	rrt = RRT(img_map=img_map2)
	
	#load initial and target position
	if GUI:
		cv2.imshow('RRT', img_map)
		cv2.setMouseCallback('RRT',rrt.pos_define, param=img_map)
		while(rrt.CLICK_COUNTER<2):
			cv2.imshow('RRT', img_map)
			if cv2.waitKey(20) & 0xFF == 27:
				break
		rrt.start_connect(img_map)
		#rrt.interpolation_path()


	#load Position from subscriber
	else:
		rrt.loadPos()
		rrt.start_connect()


	#end
	if GUI:
		while(1):
			cv2.imshow('RRT', img_map)
			if cv2.waitKey(10) & 0xFF == 27:
				break
		cv2.destroyAllWindows()
	print("quit")
