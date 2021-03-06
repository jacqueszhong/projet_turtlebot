# coding: utf-8

"""
Given a saved map file, construct a connection graph
"""


"""

.33 24 

"""
import sys
import random
import time

import numpy as np
from numpy import linalg

from collections import defaultdict
import math


def generate_random_nodes(map_data,w,h,nb_point):

	n = 0

	dict_nodes = {}
	while n < nb_point :
		y = random.randint(0,h-1)
		x = random.randint(0,w-1)

		if map_data[y][x] != 0:
			dict_nodes[n] = (y,x)
			n += 1

	return dict_nodes

def write_blank_pgm(filename):
	f = open("blank.pgm","w")

	f.write

class BuildRRT :

	_epsilonDist = 15 #Allowed error to reach target

	_d_increment = 1  #Distanceiincrements
	_nb_increment = 10 #Number of increments

	_min_dist_RRT = 50


	def __init__(self):
		print("Initializing BuildRRT")
		self._height = -1
		self._width = -1
		self._depth = -1
		self._pgm_format = ''
		

	""" Initialisation methods """
	def init(self,targetPos, initPos = (1,1),fileName = None):
		self._initPos = initPos
		self._targetPos = targetPos
		if fileName != None :
			load_pgm_map(fileName)

	def load_pgm_map(self,pgm_name):
		"""
		Load pgm file from map_saver (ROS map_saver pkg)
		"""

		im = open(pgm_name,"rt", encoding="ISO-8859-1")

		desc = 0
		line = im.readline()
		while line and desc < 2:
			if not line.startswith('#'): #ignore comments

				if desc == 0: #format
					self._pgm_format = line.split()[0]
					assert self._pgm_format == 'P5'
					desc += 1

				elif desc == 1: #dimensions
					(self._width, self._height) = [int(i) for i in line.split()]
					desc += 1

				elif desc == 2: #max value
					self._depth = int(line)
					desc += 1

				else:
					print("Nothing to do here\n")

			line = im.readline()

		#read map data
		self._map_data = []
		for y in range(self._height):
			row = []
			for x in range(self._width):
				row.append(ord(im.read(1)))
			self._map_data.append(row)

		im.close()

	def show_map(self):
		for y in range(self._height):
			for x in range(self._width):
				sys.stdout.write(str(self._map_data[y][x])+" ")
				sys.stdout.flush()
			sys.stdout.write("")


	""" RRT methods """
	def random_config(self,h,w):
		y = random.random()*(h - 1)
		x = random.random()*(w - 1)
		return (y ,x)

	def new_config(self,qnear,qrand):
		"""

		:param: qnear, tuple of pos
		:param: qrand, tuple of pos
		:param: float, maximum edge length
		"""

		#Cas points identiques
		if qrand == qnear:
			return -1

		
		#Calcul de l'incrément
		qdiff = np.array(qrand) - np.array(qnear)
		qincrement = (self._d_increment * qdiff) /linalg.norm(qdiff)

		#print("qnear={0},qrand={1},qincrement={2}".format(qnear,qrand,qincrement))

		y_new = qnear[0]
		x_new = qnear[1]
		done = False
		i = 0
		while (i < self._nb_increment) and (not done):
			y_new += qincrement[0]
			x_new += qincrement[1]
			
			#print("newy={0},newx={1},mapstatus={2}".format(y_new,x_new,self._map_data[int(y_new)][int(x_new)]))

			if self.has_collided(int(y_new),int(x_new)) : #Collision with a wall
				#print("Collided!")
				y_new -= qincrement[0]
				x_new -= qincrement[1]
				done = True
			i += 1

		return (int(y_new), int(x_new))

	def has_collided(self, y, x):
		#print("newy={0},newx={1}".format(y,x))

		if x >= self._width or x<0:
			return True
		if y >= self._height or y<0:
			return True

		if (self._map_data[y][x] == 0):
			return True

		return False


	def runRRT(self, K = -1):
		#Init
		done = False
		G = Graph()
		G.add_vertex(self._initPos)
		qnew = self._initPos

		print("Init : {0},{1}".format(self._height,self._width))


		#Run RRT until target is reached
		
		#dist = math.sqrt((qnew[0]-self._targetPos[0])**2 + (qnew[1]-self._targetPos[1])**2)
		dist = abs(qnew[0] - self._targetPos[0]) + abs(qnew[1] - self._targetPos[1])		

		c = 0
		while dist > self._epsilonDist and (not done):
			#print("NEWTURN")
			#Chose a random configuration on map
			qrand = self.random_config(self._height,self._width)

			#Get nearest vertex in graph
			idnear = G.get_nearest_vertex(qrand)
			qnear = G.get_pos_vertex(idnear)

			#Compute new configuration
			qnew = self.new_config(qnear, qrand)
			if qnew == -1 :
				#Invalid new configuration
				pass

			elif not G.has_pos(qnew) :
				idnew = G.add_vertex(qnew)
				#print("Added vertex : id={0},pos={1}".format(idnew,qnew))


				#Constructs hierarchical graph
				G.add_edge_by_id(idnew, idnear)#Vectices only know their parent vertice

				#Distance between last found node and target
				dist = abs(qnew[0] - self._targetPos[0]) + abs(qnew[1] - self._targetPos[1])

			#End condition
			K -= 1
			if K == 0 :
				done = True

			c += 1

		print("c="+str(c))		


		print("Found target ! id={0},pos={1}".format(idnew,qnew))

		#Return graph object containing all vertices and edges
		return G


class Graph :
	"""
	Graph structure for RRT
	Stores a dictionnary of edges and vertices.

	"""
	__vertex_id = 0

	def __init__(self):
		print("Initializing Graph")
		self._graph = defaultdict(set) #Dictionary {[double]:[double]} 
		self._vertices = {} #Dictionary {int:[double,double]}


	def add_vertex(self, pos):
		"""Adds a new vertex and return vertex id"""
		self._vertices[self.__vertex_id] = pos
		self.__vertex_id = self.__vertex_id + 1
		return self.__vertex_id - 1

	def get_nearest_vertex(self, pos):
		"""Returns the vertex id nearest to pos"""
		first = True
		for k,v in self._vertices.items():
			dist = abs(v[0]-pos[0]) + abs(v[1]-pos[1]) 
			#print("v={0},pos={1},dist={2}".format(v,pos,dist))
			if first :
				minDist = dist
				minId = k
				first = False

			elif (dist < minDist):
				minDist = dist
				minId = k

		return minId

	def get_pos_vertex(self, id):
		return self._vertices[id]

	def has_pos(self,pos):
		""" Tuple en entrée """
		for k,v in self._vertices.items():
			if v == pos:
				return True

		return False

	def add_edge_by_id(self, id1, id2):
		""" Adds an edge from id1 to id2 """
		self._graph[id1].add(id2)

	def get_last_id(self):
		return self.__vertex_id

	def get_neighbours(self, id):
		return self._graph[id]

	def show(self):
		for k,v in self._graph.items():
			print("Vertex {0} : {1}".format(k,v))

	def show_path(self):
		
		cur_id = self.get_last_id()-1

		print(cur_id)
		while (cur_id > 0):
			for e in self.get_neighbours(cur_id):
				print(e)
				cur_id = e

	def get_path_pos(self):
		
		list_pos = []

		cur_id = self.get_last_id()-1
		list_pos.append(self.get_pos_vertex(cur_id))

		while(cur_id)>0:
			for e in self.get_neighbours(cur_id):
				list_pos.append(self.get_pos_vertex(e))
				cur_id = e

		return list_pos


class Vertex :
	def __init__(self,pos,parent):
		self.pos = pos
		self.parent = parent

	def __repr__(self):
		print("Pos = "+str(pos)+", parent = "+str(parent))


import cv2

if __name__ == '__main__':
	rrt = BuildRRT()
	rrt.init( (200,300), initPos = (260,360) )
	rrt.load_pgm_map("test_map.pgm")

	g = rrt.runRRT()

	print(g.get_path_pos())


	img = np.zeros((512,512,3), np.uint8)
	cv2.line(img,(0,0),(511,511),(255,0,0),5)

	cv2.namedWindow("Display window",cv2.WINDOW_AUTOSIZE)
	cv2.imshow("truc",img)
	time.sleep(100)

"""
pgm_name = "../mymap.pgm"
if len(sys.argv) == 2:
	pgm_name = sys.argv[1]

map_data, width, height = load_map(pgm_name)
d = generate_random_nodes(map_data,width,height,10)

print map_data[2000][1750:2250]
print d
"""