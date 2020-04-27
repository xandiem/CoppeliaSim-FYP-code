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
#ensure coppelia is not running, then start
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
        print('YouBot\'s handle:', youbot)
floor = coppelia.get_object_handle('ResizableFloor_5_25')
print('ret:', floor)

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
	def selectRoomAtRandom(self):
		#Select the room generated at random
		random_value = randint(1,3)
		if random_value == 1:
			self.rectangular_room_generation()
		elif random_value == 2:
			self.t_room_generation()
		else:
			self.l_room_generation()
		
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

#def humanSitToStand():

#Select type of people generated
a = coppelia.create_human(-1, 0, 0, 0)
b = coppelia.create_human(1, 0, 0, 0)

#create room_gen instance
room_generator = RoomGenerator()
#sets the points of the room gen at random
room_generator.selectRoomAtRandom()
#store initial values to json
storeToJson()
int_1 = 0
point_pairs = []
for points in room_generator.room_points:
	if int_1 < len(room_generator.room_points)-1:
		point_pair = PointPair(points, room_generator.room_points[int_1+1])
		point_pairs.append(point_pair)
	elif int_1 == len(room_generator.room_points)-1:
		point_pair = PointPair(points, room_generator.room_points[0])
		point_pairs.append(point_pair)
	int_1 += 1

for pairs in point_pairs:
	pairs.printPoints()
# Start the simulation
coppelia.run_script('sim.setBoolParameter(sim.boolparam_realtime_simulation,true)')
coppelia.start()
coppelia.run_script('sim.setBoolParameter(sim.boolparam_realtime_simulation,true)')
	
last_5_seconds = time.time()-10
last_point_zero_one = time.time()-10
updates = []
timestamp = 0
value = 1
name = 'data.json'
human_path = a.setPath(point_pairs)

# Loop
while True:
    # EVERY 5 seconds
	if time.time() - last_5_seconds > 5:
		last_5_seconds = time.time()
		a.getNextPoint(human_path)
		#a.moveCircles(room_generator.room_points)
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

