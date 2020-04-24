import time
from random import randint, uniform
import random
import pygame
from coppeliasimapi import CoppeliaSimAPI, YouBot
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
		print(self.room_points[0])
		print(self.room_points[1])
		print(self.room_points[2])
		print(self.room_points[3])
	
	#call set robot to put robot in the room
		json_dictionary.update({
			"scenario":{
               		'points': [(value1/2, MAX_Y),
                	 	(-value1/2, MAX_Y),
                	    	(-value1/2, MAX_Y-value2),
                	    	(value1/2, MAX_Y-value2)],
			"robot_pose": coppelia.get_object_pose('youBot'),
			"robot_pos" : coppelia.get_object_position('youBot'),
			"human1_position": coppelia.get_object_position('Bill#0'),
			"human2_position": coppelia.get_object_position('Bill#1')
			}
       		})
	#Generate T Room
	def t_room_generation(self):
	#generate random to insert into the shape generation
		value1 = randint(60,100)/10
		top = coppelia.create_wall([(value1)/2, MAX_Y, HEIGHT],
        	            		[-(value1)/2, MAX_Y, HEIGHT])
		top_left = coppelia.create_wall([-value1/2, MAX_Y, HEIGHT],
					[-value1/2, MAX_Y-(value1/2), HEIGHT])
		top_right = coppelia.create_wall([value1/2, MAX_Y, HEIGHT],
					[value1/2,2-(value1/2), HEIGHT])
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
		print(self.room_points[0])
		print(self.room_points[1])
		print(self.room_points[2])
		print(self.room_points[3])
		print(self.room_points[4])
		print(self.room_points[5])
		print(self.room_points[6])
		print(self.room_points[7])

		json_dictionary.update({
			"scenario":{
               		'points':[(value1/2, MAX_Y),
				(-value1/2, MAX_Y),
                	  	(-value1/2, MAX_Y-value1/2),
                	   	(-value1/2+(value1/3), MAX_Y-value1/2),
                	    	(-value1/2+value1/3, MAX_Y-value1),
                	    	(value1/2-value1/3, MAX_Y-value1),
                	    	(value1/2-value1/3, MAX_Y-value1/2),
                	    	(value1/2, MAX_Y-value1/2)],
			"robot_pose": coppelia.get_object_pose('youBot'),
			"robot_pos" : coppelia.get_object_position('youBot'),
			"human1_position": coppelia.get_object_position('Bill#0'),
			"human2_position": coppelia.get_object_position('Bill#1')
			}
        	})
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
		print(self.room_points[0])
		print(self.room_points[1])
		print(self.room_points[2])
		print(self.room_points[3])
		print(self.room_points[4])
		print(self.room_points[5])
	
		json_dictionary.update({
			"scenario":{
        	        'points': [(value1/4, MAX_Y),
        	       		(-value1/4, MAX_Y),
        	            	(-value1/4, MAX_Y-value1),
        	            	(-value1/4 + value1, MAX_Y-value1),
        	            	(-value1/4+value1, MAX_Y-value1/2),
        	            	(value1/4, MAX_Y-value1/2)],
			"robot_pose": coppelia.get_object_pose('youBot'),
			"robot_pos" : coppelia.get_object_position('youBot'),
			"human1_position": coppelia.get_object_position('Bill#0'),
			"human2_position": coppelia.get_object_position('Bill#1')	
			}
        	})
def moveHumanCircles(incrementer, human):
	min_coordinates = room_generator.minimum_x_coordinates()
	min_x = min_coordinates[0]
	max_coordinates = room_generator.maximum_x_coordinates()
	max_x = max_coordinates[0]
	min_y = room_generator.minimum_y_coordinates()
	min_y = min_y[1]
	coord_1 = (0, MAX_Y-0.2, 0)
	coord_2 = (min_x/4, MAX_Y/2, 0)
	coord_3 = (min_x-0.5, MAX_Y/4, 0)
	coord_4 = (min_x/2, 0, 0)
	coord_5 = (min_x/4, min_y/4, 0)
	coord_6 = (min_x/8, min_y/2, 0)
	coord_7 = (0, min_y, 0)
	coord_8 = (max_x/8, min_y/2, 0)
	coord_9 = (max_x/4, min_y/4, 0)
	coord_10 =(max_x/2, min_y/8, 0)
	if incrementer == 1:
		human.move(coord_1[0], coord_1[1], coord_1[2])
	elif incrementer == 2:
		human.move(coord_2[0], coord_2[1], coord_2[2])
	elif incrementer == 3:
		human.move(coord_3[0], coord_3[1], coord_3[2])
	elif incrementer == 4:
		human.move(coord_4[0], coord_4[1], coord_4[2])
	elif incrementer == 5:
		human.move(coord_5[0], coord_5[1], coord_5[2])
	elif incrementer == 6:
		human.move(coord_6[0], coord_6[1], coord_6[2])
	elif incrementer == 7:
		human.move(coord_7[0], coord_7[1], coord_7[2])
	else:
		human.move(coord_8[0], coord_8[1], coord_8[2])
	
def moveHumanForwardAndBackward(human, target_val):
	#move forward
	coord_1 = (2,0,0)
	coord_2 = (-2,0,0)
	if target_val == 1:
		human.move(coord_1[0], coord_1[1], coord_1[2])
	if target_val == 2:
		human.move(coord_2[0], coord_2[1], coord_2[2])
	#move backward

#def humanSitToStand():

#Select type of people generated
a = coppelia.create_human(1, 0, 0, 0)
b = coppelia.create_human(-1, 0, 0, 0)
incrementer_a = randint(1,8)
incrementer_b = randint(1,8)
c = coppelia.create_human(0, -1, 0, 0)

#Select the room generated at random
random_value = randint(1,3)
room_generator = RoomGenerator()
if random_value == 1:
	room_generator.rectangular_room_generation()
elif random_value == 2:
	room_generator.t_room_generation()
else:
	room_generator.l_room_generation()
room_generator.minimum_x_coordinate()
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
incre_value = 1
# Loop
while True:
    # EVERY 5 seconds
	if time.time() - last_5_seconds > 5:
		last_5_seconds = time.time()
		moveHumanCircles(incrementer_a, a)
		moveHumanCircles(incrementer_b, b)
		if incrementer_a == 8:
			incrementer_a = 1
		else:
			incrementer_a += 1
		if incrementer_b == 8:
			incrementer_b = 1
		else:
			incrementer_b += 1
		moveHumanForwardAndBackward(c, incrementer_a)
		if incre_value == 1:
			incre_value += 1
		else:
			incre_value = 1
		print("value of increment a: " + str(incrementer_a))
		print("value of increment b: " + str(incrementer_b))
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

