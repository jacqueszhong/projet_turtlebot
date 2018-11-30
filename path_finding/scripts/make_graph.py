# coding: utf-8

"""
Given a saved map file, construct a connection graph
"""

import sys
import random

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

class BuildRRT :

	epsilonDist = 2 #Allowed error to reach target
	res_deltaQ = 10 #Resolution for finding delta_Q

	def __init__(self):
		print("Initializing BuildRRT")
		

	""" Initialisation methods """
	def init(self, initPos,targetPos,deltaQ,fileName = None):
		self._deltaQ = deltaQ #Incremental distance
		self._initPos = initPos
		self._targetPos = targetPos
		if fileName != None :
			load_pgm_map(fileName)

	def load_pgm_map(fileName):
		"""
		Load pgm file from map_saver (ROS map_saver pkg)

		"""
		im = open(pgm_name,"rt")

		desc = 0
		line = im.readline()
		while line and desc < 2:
			if not line.startswith('#'): #ignore comments

				if desc == 0: #format
					self.pgm_format = line.split()[0]
					assert self.pgm_format == 'P5'
					desc += 1

				elif desc == 1: #dimensions
					(self.width, self.height) = [int(i) for i in line.split()]
					desc += 1

				elif desc == 2: #max value
					self.depth = int(line)
					desc += 1

				else:
					print("Nothing to do here\n")

			line = im.readline()

		#read map data
		self.map_data = []
		for y in range(height):
			row = []
			for x in range(width):
				row.append(ord(im.read(1)))
			self.map_data.append(row)

		im.close()


	""" RRT methods """
	def random_config(self,h,w):
		y = random.randint(0,h - 1)
		x = random.randint(0,w - 1)
		return [y ,x]

	def new_config(self,qnear,qrand,deltaQ):

		qnew = qnear
		increment = deltaQ / res_deltaQ
		done = False
		while (i < res_deltaQ) or (not done):
			qnew += increment
			if (self._map[qtest[0]][qtest[1]] == 0): #Collision with a wall
				qnew -= increment
				done = True
			i += 1

		return qnew


	def runRRT(self):
		#Init
		G = Graph()
		G.add_vertex(self._initPos)
		qnew = self._initPos

		#Run RRT until target is reached
		while sqrt((qnew[0]-self._targetPos[0])**2 + (qnew[1]-self._targetPos[1])**2) > epsilonDist:

			#compute new vertice
			qrand = random_config(self._height,self.width)
			idnear = G.get_nearest_vertex(qrand)
			qnear = G.get_pos_vertex(idnear)
			qnew = new_config(qnear, qrand, self._deltaQ)
			idnew = G.add_vertex(qnew)
			G.add_edge_by_id(idnear,idnew)

			#display vertices


		

		#Return graph object containing all vertices and edges
		return G

	def show_graph(self,graph):
		


class Graph :
	"""
	Graph structure for RRT
	"""
	__vertex_id = 0

	def __init__(self):
		print("Initializing Graph")
		self._graph = defaultdict(set) #Dictionary {[int]:[int]} 
		self._vertices = {} #Dictionary {int:[int,int]}


	def add_vertex(self, pos):
		"""Adds a new vertex and return vertex id"""
		self._vertices[self.__vertex_id] = pos
		self.__vertex_id = self.__vertex_id + 1
		return self.__vertex_id - 1

	def get_nearest_vertex(self, pos):
		"""Returns the vertex id nearest to pos"""
		first = True
		for k,v in self._vertices.items():
			dist = math.sqrt((v[0]-pos[0])**2 + (v[1]-pos[1])**2) 

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

	def add_edge_by_id(self, id1, id2):
		self._graph[id1].add(id2)
		self._graph[id2].add(id1)

	def show(self):
		for k,v in self._graph.items():
			print("Vertex {0} : {1}".format(k,v))


"""
G = Graph()
G.add_vertex((1,2))
G.add_vertex((1,2.546))
G.add_vertex((1.4564,2.11))

print(G.get_nearest_vertex((1,2.0001)))
print(G.get_nearest_vertex((1.46,2.10)))
print(G.get_nearest_vertex((1.01,2.546)))
"""

"""
pgm_name = "../mymap.pgm"
if len(sys.argv) == 2:
	pgm_name = sys.argv[1]

map_data, width, height = load_map(pgm_name)
d = generate_random_nodes(map_data,width,height,10)

print map_data[2000][1750:2250]
print d
"""