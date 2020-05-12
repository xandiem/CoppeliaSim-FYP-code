import time
from random import randint, uniform
import random
import pygame
from coppeliasimapi import CoppeliaSimAPI, YouBot, PointPair
from math import pi
import re
import numpy as np
import threading
import os
import json
import os.path

#joystick setup
pygame.display.init()
pygame.joystick.init()
pygame.joystick.Joystick(0).init()
#find the models from coppelia
coppelia = CoppeliaSimAPI(['./scenes/'])
#ensure coppelia is not running
coppelia.stop()
coppelia.close()
#scene constants
HEIGHT = 0.3
MAX_Y = 4.0
json_dictionary = {}
#Load basic scene
coppelia.load_scene('dataset.ttt')
#Access scene children
children = coppelia.get_objects_children('sim.handle_scene', children_type='sim.all_type', filter_children=1+2)
for child in children:
	name = coppelia.get_object_name(child)
    # Get the youbot that is already available in the scene
	if name == 'youBot':
		youbot = YouBot(coppelia, child)

class RoomGenerator:
	def __init__(self):
		self.room_points = []
	
	def maximum_x_coordinates(self):
		maximum = max(self.room_points)[0]
		return maximum

	def minimum_x_coordinates(self):
		minimum = min(self.room_points)[0]
		return minimum

	def minimum_y_coordinates(self):
		minimum_y = min(self.room_points)[1]
		return minimum_y
		
	def pairsAreEqual(self, point_pair_1, point_pair_2):
		return point_pair_1.point_1 == point_pair_2.point_1 and point_pair_1.point_2 == point_pair_2.point_2

	def setPath(self, point_pairs_list):
        #room coordinates into pairs of coordinates(walls)
        #loop over walls
            #find walls that are parallel
		path_points = []
		for point_pair in point_pairs_list:
			for point_pair1 in point_pairs_list:
				if (not self.pairsAreEqual(point_pair, point_pair1)) and point_pair.isParallel(point_pair1) and point_pair.doesOverlap(point_pair1):
					point_pair_fixed_coordinate = 0
					point_pair1_fixed_coordinate = 0
					if point_pair.isHorizontal():
						point_pair_fixed_coordinate = point_pair.point_1[1]
						point_pair1_fixed_coordinate = point_pair1.point_1[1]
					else:
						point_pair_fixed_coordinate = point_pair.point_1[0]
						point_pair1_fixed_coordinate = point_pair1.point_1[0]   
					point_pair1_greater_than = point_pair1_fixed_coordinate > point_pair_fixed_coordinate  
					if point_pair1_greater_than:
						if point_pair.isHorizontal():
							path_points.append((point_pair.getXMin()+1, point_pair.point_1[1] + 1))
							path_points.append((point_pair.getXMax()-1, point_pair.point_1[1] + 1))
						else:
							path_points.append((point_pair.point_1[0] +1, point_pair.getYMin() + 1))
							path_points.append((point_pair.point_1[0] +1, point_pair.getYMax() - 1))
					else:
						if point_pair.isHorizontal():
							path_points.append((point_pair.getXMin()+1, point_pair.point_1[1] - 1))
							path_points.append((point_pair.getXMax()-1, point_pair.point_1[1] - 1))
						else:
							path_points.append((point_pair.point_1[0] -1, point_pair.getYMin() + 1))
							path_points.append((point_pair.point_1[0] -1, point_pair.getYMax() - 1))
		points_list_no_dupes = list(dict.fromkeys(path_points))
		sorted_points = sorted(points_list_no_dupes , key=lambda k: [k[0]])
		return sorted_points
	
	#Generate rectangular room
	def rectangular_room_generation(self):
		#generate random to insert into the shape generation		
		value1 = randint(60,100)/10		
		value2 = randint(60,100)/10
		#store set of points
		top = coppelia.create_wall([value1/2, MAX_Y, HEIGHT],
        	       			[-value1/2, MAX_Y, HEIGHT])
		left = coppelia.create_wall([-value1/2, MAX_Y, HEIGHT],
        	          		[-value1/2, MAX_Y-value2, HEIGHT])
		bottom = coppelia.create_wall([-value1/2, MAX_Y-value2, HEIGHT],
					[value1/2, MAX_Y-(value2), HEIGHT])
		right = coppelia.create_wall([value1/2, MAX_Y-(value2), HEIGHT],
					[value1/2, MAX_Y, HEIGHT])
		
		self.room_points.append((value1/2, MAX_Y))
		self.room_points.append((-value1/2, MAX_Y))
		self.room_points.append((-value1/2, MAX_Y-value2))
		self.room_points.append((value1/2, MAX_Y-value2))
	
	#Generate T Room
	def t_room_generation(self):
	#generate random to insert into the shape generation
		value1 = randint(60,100)/10
		top = coppelia.create_wall([(value1)/2, MAX_Y, HEIGHT],
        	            		[-(value1)/2, MAX_Y, HEIGHT])
		top_left = coppelia.create_wall([-value1/2, MAX_Y, HEIGHT],
					[-value1/2, MAX_Y-(value1/2), HEIGHT])
		top_right = coppelia.create_wall([value1/2, MAX_Y, HEIGHT],
					[value1/2, MAX_Y-(value1/2), HEIGHT])
		middle_left = coppelia.create_wall([-value1/2, MAX_Y-(value1/2), HEIGHT],
					[-value1/2+(value1/3), MAX_Y-value1/2, HEIGHT])
		middle_right = coppelia.create_wall([value1/2-value1/3, MAX_Y-value1/2, HEIGHT],[value1/2,MAX_Y-(value1/2), HEIGHT])
		bottom_left = coppelia.create_wall([-value1/2+(value1/3), MAX_Y-value1/2, HEIGHT], [-value1/2+value1/3, MAX_Y-value1, HEIGHT])
		bottom_right = coppelia.create_wall([value1/2-value1/3, MAX_Y-value1/2, HEIGHT],[value1/2-value1/3, MAX_Y-value1, HEIGHT])
		bottom = coppelia.create_wall([-value1/2+value1/3, MAX_Y-value1, HEIGHT],
                      		[value1/2-value1/3, MAX_Y-value1, HEIGHT])
		
		self.room_points.append((value1/2, MAX_Y))
		self.room_points.append((-value1/2, MAX_Y))
		self.room_points.append((-value1/2, MAX_Y-value1/2))
		self.room_points.append((-value1/2+(value1/3), MAX_Y-value1/2))
		self.room_points.append((-value1/2+value1/3, MAX_Y-value1))
		self.room_points.append((value1/2-value1/3, MAX_Y-value1))
		self.room_points.append((value1/2-value1/3, MAX_Y-value1/2))
		self.room_points.append((value1/2, MAX_Y-value1/2))
		
	#Generates l-room
	def l_room_generation(self):
		#generate random to insert into the shape generation
		value1 = randint(60,100)/10
		top = coppelia.create_wall([value1/4, MAX_Y, HEIGHT],
	                    		[-(value1/4), MAX_Y, HEIGHT])
		middle_left = coppelia.create_wall([-value1/4, MAX_Y, HEIGHT],
	                    		[-(value1/4), MAX_Y-(value1), HEIGHT])
		middle_right = coppelia.create_wall([value1/4, MAX_Y, HEIGHT],
	                    		[value1/4, MAX_Y-value1/2, HEIGHT])
		bottom_right = coppelia.create_wall([value1/4, MAX_Y-value1/2, HEIGHT],
					[-value1/4+value1, MAX_Y-value1/2, HEIGHT])
		right = coppelia.create_wall([-value1/4+value1, MAX_Y-value1/2, HEIGHT],
					[-value1/4 + value1, MAX_Y-value1, HEIGHT])
		bottom = coppelia.create_wall([-value1/4, MAX_Y-value1, HEIGHT],
					[-value1/4 + value1, MAX_Y-value1, HEIGHT])
		
		self.room_points.append((value1/4, MAX_Y))
		self.room_points.append((-value1/4, MAX_Y))
		self.room_points.append((-value1/4, MAX_Y-value1))
		self.room_points.append((-value1/4 + value1, MAX_Y-value1))
		self.room_points.append((-value1/4+value1, MAX_Y-value1/2))
		self.room_points.append((value1/4, MAX_Y-value1/2))

	#method to select the room to be generated
	def selectRoomAtRandom(self):
		#Select the room generated at random
		random_value = randint(1,3)
		if random_value == 1:
			self.rectangular_room_generation()
		elif random_value == 2:
			self.t_room_generation()
		else:
			self.l_room_generation()

def getRandomPoints(room_points):
	#need to use the points in the rooms to randomly place
	points = room_points
	if len(room_points) == 4:
		min_x = min(points)[0]
		max_x = max(points)[0]
		min_y = min(points)[1]
		max_y = max(points)[1]
		rand_x = uniform(min_x +0.5, max_x-0.5)
		rand_y = uniform(min_y +0.5, max_y-0.5)
		return rand_x, rand_y			
	
	elif len(room_points) == 6:
		#one list for lowest x and one for lowest y
		sorted_points_y = sorted(points, key=lambda k: [k[1], k[0]])
		sorted_points_x = sorted(points, key=lambda k: [k[0], k[1]])
		min_x_coords = sorted_points_x[0]	
		mid_x_coords = sorted_points_x[2]
		max_x_coords = sorted_points_x[5]
		#gets x from left to mid
		rand_x = uniform(min_x_coords[0]+0.5, max_x_coords[0]-0.5)
		if rand_x < mid_x_coords[0] and rand_x > min_x_coords[0]:
			rand_y = uniform(min_x_coords[1]+0.5, max_x_coords[1]-0.5)
			return rand_x, rand_y
		else:
			min_y_coords = sorted_points_y[0]
			mid_y_coords = sorted_points_y[2]
			rand_y = uniform(min_y_coords[1]+0.5, mid_y_coords[1]-0.5)
			return rand_x, rand_y
	elif len(room_points) == 8:
		sorted_points_x = sorted(points, key=lambda k: [k[0], k[1]])
		sorted_points_y = sorted(points, key=lambda k: [k[1], k[0]])
		min_x_coords = sorted_points_x[0]
		mid_x_coords = sorted_points_x[2]
		mid_rx_coords = sorted_points_x[4]
		max_x_coords = sorted_points_x[7]
		rand_x = uniform(min_x_coords[0]+0.5, max_x_coords[0]-0.5)
		if rand_x > mid_x_coords[0] and rand_x < mid_rx_coords[0]:
			min_y_coords = sorted_points_y[0]
			max_y_coords = sorted_points_y[7]
			rand_y = uniform(min_y_coords[1]+0.5, max_y_coords[1]-0.5)
			return rand_x, rand_y
		else:
			mid_y_coords = sorted_points_y[2]
			max_y_coords = sorted_points_y[7]
			rand_y = uniform(mid_y_coords[1]+0.5, max_y_coords[1]-0.5)
			return rand_x, rand_y		

	else:
		print("room is not rectangular, t-shaped or l-shaped")
		
def getHumansDictionary():
	dict_humans = {}
	# Regular expression to retrieve Bill, Bill1, etc in the scene
	pattern = '^Bill[0-9]{0,2}$'
	children = coppelia.get_objects_children('sim.handle_scene', children_type='sim.all_type', 			filter_children=1+2)
	for child in children:
		name = coppelia.get_object_name(child)
		isHuman = re.findall(pattern, name)
		if isHuman:
			pos = coppelia.get_object_position(child)
			dict_humans[str(name)] = pos
	if dict_humans:
		return dict_humans

#store key features to JSON		
def storeToJson(room_generator_points, interactions):
	dict_interaction = interactions
	points = room_generator_points
	dict_humans = getHumansDictionary()
	json_dictionary.update({
			"scenario":{
               		'points': points,
			"robot_pose": coppelia.get_object_pose('youBot'),
			"target_position" : coppelia.get_object_position('Cuboid'),
			"interactions" : dict_interaction,
			"humans" : dict_humans
			}
       		})

#sets up point pairs for use in setting room points
def populatePointPairs(room_coords):
	point_pairs = []
	int_1 = 0
	for points in room_coords:
		if int_1 < len(room_coords)-1:
			point_pair = PointPair(points, room_coords[int_1+1])
			point_pairs.append(point_pair)
		elif int_1 == len(room_coords)-1:
			point_pair = PointPair(points, room_coords[0])
			point_pairs.append(point_pair)
		int_1 += 1
	return point_pairs

#Randomly create a group of people
def createScenePeople(path, room_points):
	min = 0
	max = 9
	forward_backward_list = []
	following_humans_list = []
	interactions_dict = {}
	rand_num = randint(min, max)
	if rand_num > 0:
		count = 1
		for x in range(rand_num):
			rand_val = randint(0,3)
			if rand_val == 0:
				#get start coords and set them
				points = getRandomPoints(room_points)
				human = coppelia.create_human(points[0], points[1],0,0)
				#<0.5 to ensure both x and y inside the room
				human.setDummyPosition(-0.4, 0.2, 0)
				forward_backward_list.append(human)
				human.setPointsForwardAndBackward(path)
			elif rand_val == 1:
				#INTERACTION SET P2P- Two people moving
				points = getRandomPoints(room_points)
				human = coppelia.create_human(points[0], points[1],0,0)
				human2 = coppelia.create_human(points[0]-0.4, points[1]-0.4, 0, 0)
				human.setDummyPosition(-0.4, 0.2, 0)
				human2.setDummyPosition(-0.4, 0.2, 0)
				following_humans_list.append(human)
				following_humans_list.append(human2)
				interactions_dict["Interaction" + str(count)] = 					(coppelia.get_object_name(human.handle), 						coppelia.get_object_name(human2.handle))
				count += 1
			elif rand_val == 2:
				#INTERACTION SET P2O - CUP AND PERSON
				#sitting person on chair with cup
				points = getRandomPoints(room_points)
				human = coppelia.create_model('models/people/Sitting Bill.ttm', points[0], 						points[1], 0, 0)
				table_model = 'models/furniture/tables/customizable table.ttm'
				chair_model = 'models/furniture/chairs/dining chair.ttm'
				cup_model = 'models/household/cup.ttm'
				#+0.5x to move away from chair and person
				table = coppelia.create_model(table_model, points[0]+0.5, points[1], 0.7, 0)
				#-0.4 so the person it sitting on the chair
				chair = coppelia.create_model(chair_model, points[0]-0.4, points[1], 0.45, pi/2)
				cup = coppelia.create_model(cup_model, points[0], points[1], 1.0, 0)
				interactions_dict["Interaction" + str(count)] = 					(coppelia.get_object_name(human), coppelia.get_object_name(cup))
				count += 1
			else:
				points = getRandomPoints(room_points)
				human = coppelia.create_random_human(points[0], points[1], 0, 0)


	if not forward_backward_list and following_humans_list:
		forward_backward_list.append(1)
	elif forward_backward_list and not following_humans_list:
		following_humans_list.append(1)
	elif not forward_backward_list and not following_humans_list:
		following_humans_list.append(1)
		forward_backward_list.append(1)
	return forward_backward_list, following_humans_list, interactions_dict

def createObjects(room_points):
	#one from distance
	#one from far
	#one general
	object_list = []
	rand_value = randint(0,5)
	if rand_value > 0:
		for x in range(rand_value):
			rand_num = randint(0,2)
			if rand_num == 0:
				#tv
				tv_model = 'models/office items/laptop.ttm'
				points = getRandomPoints(room_points)
				tv = coppelia.create_model(tv_model, points[0], points[1], 0, 0)
				object_list.append(tv)
			elif rand_num == 1:
				#cupboard
				cupboard_model = 'models/furniture/shelves-cupboards-racks/deep cupboard.ttm'
				points = getRandomPoints(room_points)
				cupboard = coppelia.create_model(cupboard_model, points[0], points[1], 0.95, 0)
				object_list.append(cupboard)
	return object_list
	#random to get what to create
	

#create room_gen instance
room_generator = RoomGenerator()
#sets the points of the room gen at random
room_generator.selectRoomAtRandom()
#sets the human coordinates for movement
point_pairs = populatePointPairs(room_generator.room_points)
#get path based on pointpairs
human_path = room_generator.setPath(point_pairs)
#Generate the people for the rooms
human_lists = createScenePeople(human_path, room_generator.room_points)
createObjects(room_generator.room_points)
#ensure the lists are not empty
if human_lists[0] != 1 :
	forward_backward_list = human_lists[0]
if human_lists[1] != 1:
	around_room_list = human_lists[1]
storeToJson(room_generator.room_points, human_lists[2])
# Start the simulation
coppelia.run_script('sim.setBoolParameter(sim.boolparam_realtime_simulation,true)')
coppelia.start()
coppelia.run_script('sim.setBoolParameter(sim.boolparam_realtime_simulation,true)')
#set target position to random
target_points = getRandomPoints(room_generator.room_points)
#z set to 0.7357 as this is half the cuboid height, to ensure it is above ground
target_object = coppelia.set_object_transform('Cuboid', target_points[0], target_points[1], 0.20, 0)
last_10_seconds = time.time()-10	
last_5_seconds = time.time()-10
last_point_zero_one = time.time()-10
updates = []
timestamp = 0
value = 1
name = 'data.json'

# Loop
while True:
	# EVERY 5 seconds
	if time.time() - last_10_seconds > 10:
		last_10_seconds = time.time()
		#loop through list of forward and backward humans
		#check if new coordinate needs to be set
		if forward_backward_list[0] != 1:
			for human in forward_backward_list:
				points = human.points
				human.moveForwardAndBackwards(points)
	if time.time() - last_5_seconds > 5:
		last_5_seconds = time.time()
		#loop through list of around room humans
		#check if new coordinate needs to be set
		if around_room_list[0] != 1:
			for human in around_room_list:
				human.getNextPoint(human_path)
	# EVERY 0.01 seconds
	if time.time() - last_point_zero_one > 0.01:
		last_point_zero_one = time.time()
		pygame.event.pump()
		timestamp += 1
		dict_human = getHumansDictionary()
		updates.append({
			"timestamp": timestamp,
			"youbot_pose": coppelia.get_object_pose('youBot'),
			"humans": dict_human
		})
		left = -20*pygame.joystick.Joystick(0).get_axis(0)
		up = 20*pygame.joystick.Joystick(0).get_axis(1)
		rot = 20*pygame.joystick.Joystick(0).get_axis(2)
		youbot.set_velocity(up, left, rot)
		#get youbot and target pos
		youbot_position = coppelia.get_object_position('youBot')
		#convert to array for use in euclidean distance
		youbot_position_arr = np.array((youbot_position[0], youbot_position[1], 0))
		target_position = coppelia.get_object_position('Cuboid')
		target_position_arr = np.array((target_position[0], target_position[1], 0))
		distance_to_target = np.linalg.norm(youbot_position_arr-target_position_arr)
		if pygame.joystick.Joystick(0).get_button(0) == 1 or distance_to_target < 0.4:
			print("Exiting now, uploading data to new file...")
			json_dictionary.update({
			"updates": updates
			})
			while value <= 1000:
				if os.path.isfile(name):
					name = 'data' + str(value) + '.json'
					value += 1
				else:
					break
			with open(name, 'w') as f:
				json.dump(json_dictionary, f, indent=2)
			print("data uploaded to: " + name + '\n'+ "now exiting the program...")
			coppelia.stop()
			coppelia.close()
			exit()
	time.sleep(0.001)

