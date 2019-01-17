import sys
import random
import time

import numpy as np
from math import hypot, sqrt
import image_processing as imp
from random import randrange as rand
from numpy import linalg
import cv2

import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.misc import comb

from collections import defaultdict
import math

import lagrange as lg


GUI = 0  # activate graphic interface
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
		self._size_path=0


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

		self.update_size_path()
		if GUI:
			self.draw_path(img_map)

	def update_size_path(self):
		self._size_path = 0
		last=self._path[0]

		for coord in self._path[1:]:
			d = dist(last,coord)
			self._size_path += dist(last,coord)
			last=coord

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

	def interpolation_path_linear(self,n_points):
		""" 
		Linear interpolation of of the path
		"""
		
		if n_points==-1:
			n_points=0
		n_points -= len(self._path)
		if n_points<1 and n_points!=-1:
			return


		path=[] #new path
		total_size = self._size_path

		size_path = [] #size of each segment of the trajectory
		nb_points = [] #number of point for each segment of the trajectory
		
		## Calculate the size of each part of the trajectory

		last=self._path[0]

		for coord in self._path[1:]:
			d = dist(last,coord)
			size_path.append(d)
			last=coord
		c_points = 0

		## Calculate the number of points we will create in each part of the trajectory

		for i in range(len(size_path)-1):
			n = int(n_points*size_path[i]/total_size)
			nb_points.append(n)
			c_points += n
		nb_points.append(n_points-c_points)
		last=self._path[0]

		## Add new points

		for i in range(1,len(self._path)):
			path.append(last)			
			if nb_points != 0:
				delta_x=self._path[i][0]-last[0]
				delta_y=self._path[i][1]-last[1]
				n=nb_points[i-1]+1
				if delta_x!=0:
					for k in range(1,n):
						path.append((last[0]+k*delta_x/n,last[1]+k*delta_y/n))
				else:
					delta = (self._path[i][1]-last[1])/(nb_points[i-1]+1)
					Y= [last[1]+delta*k for k in range(1,nb_points[i-1]+1)]
					for y in Y:
						path.append((last[0],y))
			last=self._path[i]

		path.append(self._path[-1])
		self._path=path

	def interpolation_path_bezier(self):
		""" 
		Bezier curves interpolation of the path
		"""
				
		xvals,yvals=bezier_curve(self._path, nTimes=self._size_path/8)

		path= [(int(xvals[i]),int(yvals[i])) for i in range(len(xvals))]
		self._path=path

	def interpolation_path_lagrange(self):
		
		""" 
		Lagrange interpolation of the path
		"""
		x = [coord[0] for coord in self._path]
		y = [coord[1] for coord in self._path]
		step = 1
		T=[d*step+1 for d in range(0,len(x))]
		X=lg.interpole(T,x)
		Y=lg.interpole(T,y)
		path=[]
		points=20
		T=[]
		for n in range(points+1):
			T.append(1.0+n*(len(x)-1.0)*step/points)
		for i in T:
			path.append((int(lg.evalue(X,i)),int(lg.evalue(Y,i))))
		self._path=path


	def draw_path(self,img_map,color=(0,0,255)):
		"""
		Draw the path in red into the img
		"""
		for i in range(1,len(self._path)):
			img_map=cv2.line(img_map,self._path[i-1],self._path[i],color,4)
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
		global CLICK_COUNTER
		if event == cv2.EVENT_LBUTTONDBLCLK:
			if not self.collide_circle((x,y),DELTA_RADIUS):
				if CLICK_COUNTER==0:
					self._initPos = (x,y)
					param=cv2.circle(param,(x,y),DELTA_RADIUS,(0,255,0),-1)
				if CLICK_COUNTER==1:
					self._targetPos = (x,y)
					param=cv2.circle(param,(x,y),DELTA_RADIUS,(0,0,255),-1)
					#param = cv2.line(param,self._initPos,self._targetPos,0,1)
				CLICK_COUNTER +=1

	def draw_path_circle(self,img_map,color=(0,255,255)):
		for c in self._path:
			img_map=cv2.circle(img_map,c,2,color,-1)
			cv2.imshow('RRT', img_map)
			cv2.waitKey(10)
		

def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """

    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


def bezier_curve(points, nTimes=1000):
    """
       Given a set of control points, return the
       bezier curve defined by the control points.

       points should be a list of lists, or list of tuples
       such as [ [1,1], 
                 [2,3], 
                 [4,5], ..[Xn, Yn] ]
        nTimes is the number of time steps, defaults to 1000

        See http://processingjs.nihongoresources.com/bezierinfo/
    """

    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)

    return xvals, yvals



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
		while(CLICK_COUNTER<2):
			cv2.imshow('RRT', img_map)
			if cv2.waitKey(20) & 0xFF == 27:
				break
		print("Running RRT connect")
		rrt.start_connect(img_map)


	#load Position from subscriber
	else:
		rrt.loadPos()
		rrt.start_connect()

	print(rrt._size_path)
	rrt.interpolation_path_linear(int(rrt._size_path/4))
	rrt.draw_path_circle(img_map)

	rrt.interpolation_path_bezier()
	rrt.draw_path(img_map,(255,0,255))
	

	#end
	if GUI:
		while(1):
			cv2.imshow('RRT', img_map)
			if cv2.waitKey(10) & 0xFF == 27:
				break
		cv2.destroyAllWindows()
	print("quit")
