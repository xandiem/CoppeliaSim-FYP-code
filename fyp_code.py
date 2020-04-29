import time
from random import randint, uniform
import random
import pygame
from coppeliasimapi import CoppeliaSimAPI, YouBot, PointPair
from math import pi
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
floor = coppelia.get_object_handle('ResizableFloor_5_25')

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
		middle_right = coppelia.create_wall([value1/2-value1/3, MAX_Y-value1/2, 					HEIGHT],[value1/2,MAX_Y-(value1/2), HEIGHT])
		bottom_left = coppelia.create_wall([-value1/2+(value1/3), MAX_Y-value1/2, 				HEIGHT], [-value1/2+value1/3, MAX_Y-value1, HEIGHT])
		bottom_right = coppelia.create_wall([value1/2-value1/3, MAX_Y-value1/2, 				HEIGHT],[value1/2-value1/3, MAX_Y-value1, HEIGHT])
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

#store key features to JSON		
def storeToJson():
	points = room_generator.room_points
	json_dictionary.update({
			"scenario":{
               		'points': points,
			"robot_pose": coppelia.get_object_pose('youBot'),
			"robot_pos" : coppelia.get_object_position('youBot'),
			"human1_position": coppelia.get_object_position('Bill#0'),
			"human2_position": coppelia.get_object_position('Bill#1')
			}
       		})

#sets up point pairs for use in setting room points
def populatePointPairs(room_points):
	point_pairs = []
	int_1 = 0
	for points in room_points:
		if int_1 < len(room_points)-1:
			point_pair = PointPair(points, room_points[int_1+1])
			point_pairs.append(point_pair)
		elif int_1 == len(room_points)-1:
			point_pair = PointPair(points, room_points[0])
			point_pairs.append(point_pair)
		int_1 += 1
	return point_pairs

#Randomly create a group of people
def createScenePeople():
	min = 0
	max = 9
	forward_backward_list = []
	around_room_list = []
	rand_num = randint(min, max)
	if rand_num > 0:
		for x in range(rand_num):
			print(x)
			rand_val = randint(0,2)
			if rand_val == 0:
				human = coppelia.create_human(0,0,0,0)
				forward_backward_list.append(human)
				human.setPointsForwardAndBackward(human_path)
			elif rand_val == 1:
				human = coppelia.create_human(0,0,0,0)
				around_room_list.append(human)
			else:
				coppelia.create_model('models/people/Walking Bill.ttm')				

		return forward_backward_list, around_room_list	
			

#create room_gen instance
room_generator = RoomGenerator()
#sets the points of the room gen at random
room_generator.selectRoomAtRandom()
#sets the human coordinates for movement
point_pairs = populatePointPairs(room_generator.room_points)
#create humans
a = coppelia.create_human(0.5, 1, 0, 0)
#get path based on pointpairs
human_path = a.setPath(point_pairs)
b = coppelia.create_human(0, 1, 0, 0)

human_lists = createScenePeople()
forward_backward_list = human_lists[0]
around_room_list = human_lists[1]

values = b.selectPointsAtRandom(human_path)
print(values)
#store initial values to json
storeToJson()

# Start the simulation
coppelia.run_script('sim.setBoolParameter(sim.boolparam_realtime_simulation,true)')
coppelia.start()
coppelia.run_script('sim.setBoolParameter(sim.boolparam_realtime_simulation,true)')

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
		if forward_backward_list:
			for human in forward_backward_list:
				values = human.points
				human.moveForwardAndBackwards(values)
	if time.time() - last_5_seconds > 5:
		last_5_seconds = time.time()
		#loop through list of around room humans
		#check if new coordinate needs to be set
		if around_room_list:
			for human in around_room_list:
				human.getNextPoint(human_path)
	# EVERY 0.01 seconds
	if time.time() - last_point_zero_one > 0.01:
		last_point_zero_one = time.time()
		pygame.event.pump()
		timestamp += 1
		updates.append({
			"timestamp": timestamp,
			"youbot_pose": coppelia.get_object_pose('youBot'),
			"youbot_pos": coppelia.get_object_position('youBot'),
			"human1_pos": coppelia.get_object_position('Bill#0'),
			"human2_pos": coppelia.get_object_position('Bill#1')
		})
		left = -20*pygame.joystick.Joystick(0).get_axis(0)
		up = 20*pygame.joystick.Joystick(0).get_axis(1)
		rot = 20*pygame.joystick.Joystick(0).get_axis(2)
		youbot.set_velocity(up, left, rot)
		if pygame.joystick.Joystick(0).get_button(0) == 1:
			print("Exit button pressed, uploading data to new file...")
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

