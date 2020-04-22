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

json_dictionary = {}
#height const for room generation
HEIGHT = 0.3
MAX_Y = 2.0
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

#Generate rectangle room
def rectangular_room_generation():
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

	"""value_range1 = uniform(-10*(value1/2), 10*(value1/2))/10
	value_range2 = uniform(10*(MAX_Y), 10*(MAX_Y-value2))/10
	coppelia.set_object_transform('Bill#0', value_range1, value_range2, 0, 0)
	value_range1 = uniform(-10*(value1/2), 10*(value1/2))/10
	value_range2 = uniform(10*(MAX_Y), 10*(MAX_Y-value2))/10
	coppelia.set_object_transform('Bill#1', value_range1, value_range2, 0, 0)
	"""

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
def t_room_generation():
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
	middle_right = coppelia.create_wall([value1/2-value1/3, MAX_Y-value1/2, HEIGHT],
                     			[value1/2,MAX_Y-(value1/2), HEIGHT])
	bottom_left = coppelia.create_wall([-value1/2+(value1/3), MAX_Y-value1/2, HEIGHT], [-value1/2+value1/3, MAX_Y-value1, HEIGHT])
	bottom_right = coppelia.create_wall([value1/2-value1/3, MAX_Y-value1/2, HEIGHT],
                    			[value1/2-value1/3, MAX_Y-value1, HEIGHT])
	bottom = coppelia.create_wall([-value1/2+value1/3, MAX_Y-value1, HEIGHT],
                      		[value1/2-value1/3, MAX_Y-value1, HEIGHT])
	
	#Set human position to be within room at random
	"""value_range1 = uniform(-10*(value1/2), 10*(value1/2))/10
	value_range2 = uniform(10*(MAX_Y), 10*(MAX_Y-value1/2))/10
	coppelia.set_object_transform('Bill#0', value_range1, value_range2, 0, 0)
	value_range1 = uniform(-10*(value1/2), 10*(value1/2))/10
	value_range2 = uniform(10*(MAX_Y), 10*(MAX_Y-value1/2))/10
	coppelia.set_object_transform('Bill#1', value_range1, value_range2, 0, 0)
	"""

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
def l_room_generation():
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
	#points recorded starting from top right corner- anti clockwise round

	#Set human position to be within room at random
	"""value_range1 = uniform(-10*(value1/4), 10*(value1/4))/10
	value_range2 = uniform(10*(MAX_Y), 10*(MAX_Y-value1))/10
	coppelia.set_object_transform('Bill#0', value_range1, value_range2, 0, 0)
	value_range3 = uniform(-10*value1/4, 10*value1/4)/10
	value_range4 = uniform(10*MAX_Y, 10*MAX_Y-value1)/10
	coppelia.set_object_transform('Bill#1', value_range3, value_range4, 0, 0)"""
	
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
	coord_1 = (0,1.5,0)
	coord_2 = (-0.75, 0.75, 0)
	coord_3 = (-1.5,0,0)
	coord_4 = (-0.75, -0.75, 0)
	coord_5 = (0,-1.5,0)
	coord_6 = (0.75, -0.75, 0)
	coord_7 = (1.5,0,0)
	coord_8 = (0.75, 0.75, 0)
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

def storeAndExit(data_store):
	print("Exit button pressed, uploading data to new file...")
	json_dictionary.update({
		"updates": date_store
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
#Select type of people generated
a = coppelia.create_human(0, 0, 0, 0)
b = coppelia.create_human(0, 0, 0, 0)
incrementer_a = randint(1,8)
incrementer_b = randint(1,8)
c = coppelia.create_human(0, 0, 0, 0)

#Select the room generated at random
random_value = randint(1,3)
if random_value == 1:
	rectangular_room_generation()
elif random_value == 2:
	t_room_generation()
else:
	l_room_generation()
 	
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
			storeAndExit(updates)
	time.sleep(0.001)

