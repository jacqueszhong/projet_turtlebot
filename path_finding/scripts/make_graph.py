# coding: utf-8

"""
Given a saved map file, construct a connection graph
"""

import sys
import random

def load_map(pgm_name):
	"""
	Load pgm file from map_saver (ROS map_saver pkg)

	"""
	im = open(pgm_name,"rt")

	desc = 0
	line = im.readline()
	while line and desc < 2:
		if not line.startswith('#'): #ignore comments

			if desc == 0: #format
				pgm_format = line.split()[0]
				assert pgm_format == 'P5'
				desc += 1

			elif desc == 1: #dimensions
				(width, height) = [int(i) for i in line.split()]
				desc += 1

			elif desc == 2: #max value
				depth = int(line)
				desc += 1

			else:
				print("Nothing to do here\n")

		line = im.readline()

	#read map data
	map_data = []
	for y in range(height):
		row = []
		for x in range(width):
			row.append(ord(im.read(1)))
		map_data.append(row)

	im.close()

	return (map_data,width,height)

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



pgm_name = "../mymap.pgm"
if len(sys.argv) == 2:
	pgm_name = sys.argv[1]

map_data, width, height = load_map(pgm_name)
d = generate_random_nodes(map_data,width,height,10)

print map_data[2000][1750:2250]
print d