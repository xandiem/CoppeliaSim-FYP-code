#!/usr/bin/env python3
import os
import b0RemoteApi
import sys
import signal
import numpy as np
from math import cos, sin, atan2, pi
from random import randint
from os import path
import time


def please_exit(sig, frame):
	sys.exit(0)


def get_transform_matrix(x, y, z, angle):
	rotate_matrix = np.matrix([[cos(angle), -sin(angle), 0., 0.],
	                           [sin(angle), cos(angle), 0., 0.],
	                           [0., 0., 1., 0.],
	                           [0., 0., 0., 1.]])
	translate_matrix = np.matrix([[1., 0., 0., x],
	                              [0., 1., 0., y],
	                              [0., 0., 1., z],
	                              [0., 0., 0., 1.]])
	return (translate_matrix * rotate_matrix).flatten().tolist()[0]


#
# Generic Handle
#
class CoppeliaHandle(object):
	def __init__(self, coppelia, handle):
		super(CoppeliaHandle, self).__init__()
		self.c = coppelia
		self.handle = handle


# Wall
#  
class Wall(CoppeliaHandle):
	def __init__(self, coppelia, handle):
		super(Wall, self).__init__(coppelia, handle)


# Class of pairs of points
class PointPair:
	# pair of points as its fields
	point_1 = (0, 0, 0)
	point_2 = (0, 0, 0)

	def __init__(self, point_1, point_2):
		self.point_1 = point_1
		self.point_2 = point_2

	# Checks if two PointPairs are parallel
	# Input: point_pair: PointPair object from the room
	# Outputs: returns true if parallel
	def isParallel(self, point_pair):
		return self.isHorizontal() == point_pair.isHorizontal()

	# Checks if two points are horizontal
	# Input: NONE
	# Outputs: returns true if horizontal, false otherwise
	def isHorizontal(self):
		return self.point_1[1] == self.point_2[1]

	# Gets max X from the two points
	# Input: NONE
	# Outputs: returns highest x value
	def getXMax(self):
		if self.point_1[0] > self.point_2[0]:
			return self.point_1[0]
		else:
			return self.point_2[0]

	# Gets min X from the two points
	# Input: NONE
	# Outputs: returns minimum x value
	def getXMin(self):
		if self.point_1[0] < self.point_2[0]:
			return self.point_1[0]
		else:
			return self.point_2[0]

	# Gets max Y from the two points
	# Input: NONE
	# Outputs: returns highest Y value
	def getYMax(self):
		if self.point_1[1] > self.point_2[1]:
			return self.point_1[1]
		else:
			return self.point_2[1]

	# Gets min Y from the two points
	# Input: NONE
	# Outputs: returns minimum Y value
	def getYMin(self):
		if self.point_1[1] < self.point_2[1]:
			return self.point_1[1]
		else:
			return self.point_2[1]

	# Checks if two pairs of points overlap in the room
	# Used to check to ensure they are the complex points in the L and T rooms
	# Input: point_pair: PointPair
	# Outputs: returns minimum Y value
	def doesOverlap(self, point_pair):
		if self.isHorizontal():
			points_pair1 = []
			points_pair1.append(self.point_1[0])
			points_pair1.append(self.point_2[0])
			pair1_min_x = min(points_pair1)
			pair1_max_x = max(points_pair1)
			points_pair2 = []
			points_pair2.append(point_pair.point_1[0])
			points_pair2.append(point_pair.point_2[0])
			pair2_min_x = min(points_pair2)
			pair2_max_x = max(points_pair2)
			if (pair2_min_x > pair1_min_x and pair2_min_x < pair1_max_x):
				return True
			elif (pair2_max_x < pair1_max_x and pair2_max_x > pair1_min_x):
				return True
			elif (pair1_min_x > pair2_min_x and pair1_min_x < pair2_max_x):
				return True
			elif (pair1_max_x < pair2_max_x and pair1_max_x > pair2_min_x):
				return True
			elif (pair2_max_x == pair1_max_x and pair2_min_x == pair1_min_x):
				return True
		else:
			points_pair1 = []
			points_pair1.append(self.point_1[1])
			points_pair1.append(self.point_2[1])
			pair1_min_y = min(points_pair1)
			pair1_max_y = max(points_pair1)
			points_pair2 = []
			points_pair2.append(point_pair.point_1[1])
			points_pair2.append(point_pair.point_2[1])
			pair2_min_y = min(points_pair2)
			pair2_max_y = max(points_pair2)
			if (pair2_min_y > pair1_min_y and pair2_min_y < pair1_max_y):
				return True
			elif (pair2_max_y < pair1_max_y and pair2_max_y > pair1_min_y):
				return True
			elif (pair1_min_y > pair2_min_y and pair1_min_y < pair2_max_y):
				return True
			elif (pair1_max_y < pair2_max_y and pair1_max_y > pair2_min_y):
				return True
			elif (pair2_max_y == pair1_max_y and pair2_min_y == pair1_min_y):
				return True
		return False


# Human
#  
class Human(CoppeliaHandle):
	def __init__(self, coppelia: 'CoppeliaSimAPI', handle: int, points):
		super(Human, self).__init__(coppelia, handle)
		self.dummy_handle = coppelia.get_objects_children(handle, 'sim.object_dummy_type')[0]
		self.points = []

	# Sets the human target position based on the inputted coordinates
	# Input: x, y, z: set of coordinates
	# Outputs: NONE
	def move(self, x, y, z):
		self.c.set_object_position(self.dummy_handle, x, y, z)

	# Checks a pair of points are not identical
	# Input: point_pair_1: PointPair, point_pair_2: PointPair
	# Outputs: True if both points the same, false otherwise
	def pairsAreEqual(self, point_pair_1, point_pair_2):
		return point_pair_1.point_1 == point_pair_2.point_1 and point_pair_1.point_2 == point_pair_2.point_2

	# This method is used to get all the interior points for the human outskirts movement
	# It returns a list of interior points
	# Input: point_pairs_list: List of all PointPairs
	# Outputs: sorted_points: List of interior points
	def setPath(self, point_pairs_list):
		# room coordinates into pairs of coordinates(walls)
		# loop over all PointPairs
		# find PointPairs that are parallel
		path_points = []
		for point_pair in point_pairs_list:
			for point_pair1 in point_pairs_list:
				#check pairs are not equal, that they are parallel and they do overlap
				if (not self.pairsAreEqual(point_pair, point_pair1)) and point_pair.isParallel(
					point_pair1) and point_pair.doesOverlap(point_pair1):
					point_pair_fixed_coordinate = 0
					point_pair1_fixed_coordinate = 0
					#If horizontal
					if point_pair.isHorizontal():
						point_pair_fixed_coordinate = point_pair.point_1[1]
						point_pair1_fixed_coordinate = point_pair1.point_1[1]
					else:
						point_pair_fixed_coordinate = point_pair.point_1[0]
						point_pair1_fixed_coordinate = point_pair1.point_1[0]
					point_pair1_greater_than = point_pair1_fixed_coordinate > point_pair_fixed_coordinate
					if point_pair1_greater_than:
						if point_pair.isHorizontal():
							#add the interior points to the list
							path_points.append(
								(point_pair.getXMin() + 1, point_pair.point_1[1] + 1))
							# add the interior points to the list
							path_points.append(
								(point_pair.getXMax() - 1, point_pair.point_1[1] + 1))
						else:
							path_points.append(
								(point_pair.point_1[0] + 1, point_pair.getYMin() + 1))
							path_points.append(
								(point_pair.point_1[0] + 1, point_pair.getYMax() - 1))
					else:
						if point_pair.isHorizontal():
							path_points.append(
								(point_pair.getXMin() + 1, point_pair.point_1[1] - 1))
							path_points.append(
								(point_pair.getXMax() - 1, point_pair.point_1[1] - 1))
						else:
							path_points.append(
								(point_pair.point_1[0] - 1, point_pair.getYMin() + 1))
							path_points.append(
								(point_pair.point_1[0] - 1, point_pair.getYMax() - 1))
		points_list_no_dupes = list(dict.fromkeys(path_points))
		sorted_points = sorted(points_list_no_dupes, key=lambda k: [k[0]])
		return sorted_points

	# Sets the next point in the path when called
	# Uses euclidean distance to check which point the target is at
	# This method sets the next point in the list
	# Input: path_points: interior points list
	# Outputs: NONE
	def setNextPointOnPath(self, path_points):
		# gets target pos
		curr_pos = self.c.get_object_position(self.dummy_handle)
		curr_pos_array = np.array((curr_pos[0], curr_pos[1], curr_pos[2]))
		value = 0
		new_coords = (0, 0)
		for points in path_points:
			coordinates = (points[0], points[1], 0)
			distance_to_target = np.linalg.norm(curr_pos_array - coordinates)
			# compare target to points
			if distance_to_target < 0.3:
				if (value == (len(path_points) - 1)):
					new_coords = path_points[0]
					break
				elif (value < len(path_points) - 1):
					new_coords = path_points[value + 1]
					break
			else:
				new_coords = path_points[0]
			value += 1
		position_target = self.c.set_object_position(self.dummy_handle, new_coords[0], new_coords[1], 0)

	# Gets next point and sets it
	# Check to ensure human is close enough to target before doing so
	# Input: path_points: interior points list
	# Outputs: NONE
	def getNextPoint(self, path_points):
		# make sure human is at the curr target pos
		# loop and check if human pos is within range
		for points in path_points:
			position_human = self.c.get_object_position(self.handle)
			human_pos = np.array((position_human[0], position_human[1], position_human[2]))
			position_target = np.array((self.c.get_object_position(self.dummy_handle)))
			distance_to_target = np.linalg.norm(position_target - human_pos)
			# 0.3 to ensure no bugs
			if (distance_to_target < 0.3):
				self.setNextPointOnPath(path_points)
				break

	# Sets the next point in the forward and backward movement method
	# Checks distance, if close enough then sets to the other point
	# Uses euclidean distance to check if the human close enough to the target
	# Input: points: two points to alternate between
	# Outputs: NONE
	def moveForwardAndBackwards(self, points):
		# takes 2 different points
		coords_1 = points[0]
		coords_1 = np.array((coords_1[0], coords_1[1], 0))
		coords_2 = points[1]
		coords_2 = np.array((coords_2[0], coords_2[1], 0))
		position_human = self.c.get_object_position(self.handle)
		human_pos = np.array((position_human[0], position_human[1], position_human[2]))
		position_target = np.array((self.c.get_object_position(self.dummy_handle)))
		distance_to_target = np.linalg.norm(position_target - human_pos)
		from_coords_1_to_target = np.linalg.norm(position_target - coords_1)
		from_coords_2_to_target = np.linalg.norm(position_target - coords_2)
		if distance_to_target < 0.3:
			if from_coords_1_to_target < 0.2:
				self.c.set_object_position(self.dummy_handle, coords_2[0], coords_2[1], 0)
			else:
				self.c.set_object_position(self.dummy_handle, coords_1[0], coords_1[1], 0)

	# Gets two points for forward and backward movement
	# Uses interior points to do this
	# Input: path_points: interior points list
	# Outputs: pair of points
	def selectPointsAtRandom(self, path_points):
		for points in path_points:
			rand_1 = randint(0, len(path_points) - 1)
			rand_2 = randint(0, len(path_points) - 1)
			if not rand_1 == rand_2:
				return path_points[rand_1], path_points[rand_2]
				break

	# Sets the two points in the forward and backward method
	# Input: path_points: interior points list
	# Outputs: NONE
	def setPointsForwardAndBackward(self, path_points):
		points = self.selectPointsAtRandom(path_points)
		self.points.append(points[0])
		self.points.append(points[1])

	# Gets the two points for forward and backward method
	# Input: NONE
	# Outputs: points: pair of points
	def getPointsForwardAndBackward(self):
		return self.points

	# Sets the target position for the human
	# Input: x,y,z: set of coordinates
	# Outputs: NONE
	def setDummyPosition(self, x, y, z):
		self.c.set_object_position(self.dummy_handle, x, y, z)
#
#Class for randomyl moving human
class RandomHuman(CoppeliaHandle):
	def __init__(self, coppelia: 'CoppeliaSimAPI', handle: int):
		super(RandomHuman, self).__init__(coppelia, handle)
#
# YouBot
#
class YouBot(CoppeliaHandle):
	def __init__(self, coppelia: 'CoppeliaSimAPI', handle: int):
		super(YouBot, self).__init__(coppelia, handle)
		children = coppelia.get_objects_children(handle, children_type='sim_object_joint_type',
		                                         filter_children=0)
		for h in children:
			name = self.c.get_object_name(h)
			if 'rollingJoint_fl' in name:
				# print('WHEEL 1:', h, name)
				self.wheel1 = h
			elif 'rollingJoint_rl' in name:
				# print('WHEEL 2:', h, name)
				self.wheel2 = h
			elif 'rollingJoint_rr' in name:
				# print('WHEEL 3:', h, name)
				self.wheel3 = h
			elif 'rollingJoint_fr' in name:
				# print('WHEEL 4:', h, name)
				self.wheel4 = h

	@staticmethod
	def get_position_offsets():
		return 0., 0., 0.095

	@staticmethod
	def get_orientation_offsets():
		return 0., -1.57, 0.

	def set_velocity(self, forwBackVel, leftRightVel, rotVel):
		"""wheel_radius = 47.5/(1000*2)
#velocity in M/s
forwBackVel /= wheel_radius
leftRightVel /= wheel_radius
rotVel /= wheel_radius
self.c.set_joint_target_velocity(self.wheel1, -forwBackVel-leftRightVel-rotVel)
self.c.set_joint_target_velocity(self.wheel2, -forwBackVel+leftRightVel-rotVel)
self.c.set_joint_target_velocity(self.wheel3, -forwBackVel-leftRightVel+rotVel)
self.c.set_joint_target_velocity(self.wheel4, -forwBackVel+leftRightVel+rotVel)
"""
		self.c.set_joint_target_velocity(self.wheel1, -forwBackVel - leftRightVel - rotVel)
		self.c.set_joint_target_velocity(self.wheel2, -forwBackVel + leftRightVel - rotVel)
		self.c.set_joint_target_velocity(self.wheel3, -forwBackVel - leftRightVel + rotVel)
		self.c.set_joint_target_velocity(self.wheel4, -forwBackVel + leftRightVel + rotVel)


#
# Pioneer_p3dx
#
class Pioneer_p3dx(CoppeliaHandle):
	def __init__(self, coppelia: 'CoppeliaSimAPI', handle: int):
		super(Pioneer_p3dx, self).__init__(coppelia, handle)
		children = coppelia.get_objects_children(handle, children_type='sim_object_joint_type',
		                                         filter_children=0)
		for h in children:
			name = self.c.get_object_name(h)
			if 'Pioneer_p3dx_leftMotor' in name:
				# print('left:', h, name)
				self.left_motor = h
			elif 'Pioneer_p3dx_rightMotor' in name:
				# print('right:', h, name)
				self.right_motor = h

	@staticmethod
	def get_position_offsets():
		return 0., 0., 0.095

	@staticmethod
	def get_orientation_offsets():
		return 0., 0., 0.

	def set_velocity(self, adv, rot):
		# print('@PIONEER', adv, rot)
		axisLength = 0.381
		wheelRadius = 0.0975
		left = (adv - (rot * axisLength) / 2.) / wheelRadius
		right = (adv + (rot * axisLength) / 2.) / wheelRadius
		self.c.set_joint_target_velocity(self.left_motor, left)
		self.c.set_joint_target_velocity(self.right_motor, right)

# CoppeliaSimAPI
class CoppeliaSimAPI(object):
	def __init__(self, paths=[]):
		super(CoppeliaSimAPI, self).__init__()
		self.coppelia_paths = paths + ['./', os.environ['COPPELIASIM_ROOT'] + '/']
		self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient', 'b0RemoteApiAddOn')
		signal.signal(signal.SIGINT, please_exit)

	def load_scene(self, scene_path):
		for source in self.coppelia_paths:
			full_path = source + '/' + scene_path
			if path.exists(full_path):
				return self.client.simxLoadScene(os.path.abspath(full_path),
				                                 self.client.simxServiceCall())

	def start(self):
		self.client.simxStartSimulation(self.client.simxServiceCall())

	def stop(self):
		self.client.simxStopSimulation(self.client.simxServiceCall())

	def get_objects_children(self, parent='sim.handle_scene', children_type=None, filter_children=1 + 2):
		if children_type is None:
			children_type = 'sim.handle_all'
		ret = self.client.simxGetObjectsInTree(parent, children_type, filter_children,
		                                       self.client.simxServiceCall())
		# print('Got children query {}.'.format(ret))
		if ret[0] is True:
			return ret[1]
		raise Exception('Error getting human\'s children {}.'.format(parent))

	def set_object_parent(self, obj, parent, keep_in_place=True):
		obj = self.convert_to_valid_handle(obj)
		parent = self.convert_to_valid_handle(parent)
		code = "sim.setObjectParent({}, {}, {})".format(obj, parent, keep_in_place)
		print(code)
		print(code)
		print(code)
		ret = self.run_script(code)
		print(ret)
		return ret

	def get_object_name(self, handle, alternative_name=''):
		obj = self.convert_to_valid_handle(handle)
		ret = self.client.simxGetObjectName(handle, alternative_name, self.client.simxServiceCall())
		if ret[0] is True:
			return ret[1]
		raise Exception('Can\'t get name for object handle {}.'.format(handle))

	def create_human(self, x, y, z, angle):
		model = 'models/people/path planning Bill.ttm'
		human_handle = self.create_model(model, x, y, z, angle)
		# print('Got human handle {}.'.format(human_handle))
		return Human(self, human_handle, points=None)

	def create_random_human(self, x, y, z, angle):
		model = 'models/people/Walking Bill.ttm'
		human_handle = self.create_model(model, x, y, z, angle)
		return RandomHuman(self, human_handle)

	def create_wall(self, p1, p2):
		# pre
		model = 'models/infrastructure/walls/80cm high walls/wall section 100cm.ttm'
		x, y = 0.5 * (p1[0] + p2[0]), 0.5 * (p1[1] + p2[1])
		angle = atan2(p2[1] - p1[1], p2[0] - p1[0])
		# print(angle)
		length = np.linalg.norm(np.array(p2) - np.array(p1))
		# create
		wall_handle = self.create_model(model, x, y, p1[2], angle)
		# resize
		child = self.get_objects_children(wall_handle, 'sim.object_shape_type')[0]
		# print('Got child wall handle {}.'.format(child))
		self.scale_object(child, 6.749 * length, 0.12, 1.5)
		self.scale_object(wall_handle, 6.749 * length, 0.12, 1.5)
		return Wall(self, wall_handle)

	def create_model(self, model, x=None, y=None, z=None, rz=None):
		for source in self.coppelia_paths:
			full_path = source + '/' + model
			if path.exists(full_path):
				# print ('File "{}" exists: {}'.format(model, full_path))
				ret = self.client.simxLoadModelFromFile(os.path.abspath(full_path),
				                                        self.client.simxServiceCall())
				# print('Result of creating the model {}'.format(ret))
				if ret[0] is not True:
					raise Exception('Error creating model {}.'.format(full_path))
				# print('Created model with handle {}.'.format(ret[1]))
				if x is not None and y is not None and z is not None:
					self.set_object_transform(ret[1], x, y, z, rz)
				return ret[1]
		raise Exception('Couldn\'t find model {} in any of the paths {}'.format(model, self.coppelia_paths))

	def get_object_handle(self, object_name):
		obj = self.client.simxGetObjectHandle(object_name, self.client.simxServiceCall())
		if obj[0] is True:
			return obj[1]
		raise Exception('Can\'t get object handle for object {}'.format(object_name))

	def set_object_transform(self, obj, x, y, z, angle):
		obj = self.convert_to_valid_handle(obj)
		M = get_transform_matrix(x, y, z, angle)
		ret = self.client.simxSetObjectMatrix(obj, -1, M, self.client.simxServiceCall())
		return ret

	def convert_to_valid_handle(self, obj):
		if isinstance(obj, CoppeliaHandle):
			obj = obj.handle
		elif type(obj) is str:
			if obj not in ['sim.handle_parent']:
				obj = self.client.simxGetObjectHandle(obj, self.client.simxServiceCall())[1]
		return obj

	def set_object_position(self, obj, x, y, z, reference='sim.handle_parent'):
		obj = self.convert_to_valid_handle(obj)
		reference = self.convert_to_valid_handle(reference)
		ret = self.client.simxSetObjectPosition(obj, reference, [x, y, z], self.client.simxServiceCall())
		return ret

	def get_object_position(self, obj, reference=-1):
		obj = self.convert_to_valid_handle(obj)
		reference = self.convert_to_valid_handle(reference)
		ret = self.client.simxGetObjectPosition(obj, reference, self.client.simxServiceCall())
		if ret[0] is not True:
			raise Exception('CoppeliaSimAPI: get_object_position: Can\'t find object {}.'.format(obj))
		return ret[1]

	def get_object_pose(self, obj, reference=-1):
		obj = self.convert_to_valid_handle(obj)
		reference = self.convert_to_valid_handle(reference)
		ret = self.client.simxGetObjectPose(obj, reference, self.client.simxServiceCall())
		if ret[0] is not True:
			raise Exception('CoppeliaSimAPI: get_object_pose: Can\'t find object {}.'.format(obj))
		return ret[1]

	def set_object_orientation(self, obj, x, y, z, reference='sim.handle_parent'):
		obj = self.convert_to_valid_handle(obj)
		reference = self.convert_to_valid_handle(reference)
		return self.client.simxSetObjectOrientation(obj, reference, [x, y, z], self.client.simxServiceCall())

	def get_object_orientation(self, obj, reference=-1):
		obj = self.convert_to_valid_handle(obj)
		reference = self.convert_to_valid_handle(reference)
		ret = self.client.simxGetObjectOrientation(obj, reference, self.client.simxServiceCall())
		if ret[0] is not True:
			raise Exception('CoppeliaSimAPI: get_object_orientation: Can\'t find object {}.'.format(obj))
		return ret[1]

	def remove_object(self, obj):
		if type(obj) is str:
			sobj = self.client.simxGetObjectHandle(obj, self.client.simxServiceCall())[1]
			# print('Converted {} to {}.'.format(obj, sobj))
			obj = sobj
		return self.client.simxRemoveObjects([obj], 1 + 2, self.client.simxServiceCall())

	def run_script(self, script):
		return self.client.simxExecuteScriptString(script, self.client.simxServiceCall())

	def scale_object(self, handle, sx, sy, sz):
		return self.run_script('sim.scaleObject({},{},{},{},0)'.format(handle, sx, sy, sz))

	def close(self):
		return self.client.simxCloseScene(self.client.simxServiceCall())

	# NOT INCLUDED IN THE DOCUMENTATION YET
	def get_youbot(self) -> YouBot:
		children = self.get_objects_children('sim.handle_scene', children_type='sim.object_shape_type', filter_children=1 + 2)
		for h in children:
			name = self.get_object_name(h)
			if name == 'youBot':
				return YouBot(self, h)

	def create_youbot(self, x, y, z) -> YouBot:
		ix, iy, iz = YouBot.get_position_offsets()
		ret = self.create_model('models/robots/mobile/KUKA YouBot.ttm', x + ix, y + iy, z + iz, 0.)
		self.set_object_orientation(ret, *YouBot.get_orientation_offsets())
		return YouBot(self, ret)

	def create_pioneer_p3dx(self, x, y, z) -> Pioneer_p3dx:
		ix, iy, iz = Pioneer_p3dx.get_position_offsets()
		ret = self.create_model('models/robots/mobile/pioneer p3dx.ttm', x + ix, y + iy, z + iz, 0.)
		self.set_object_orientation(ret, *Pioneer_p3dx.get_orientation_offsets())
		return Pioneer_p3dx(self, ret)

	def get_pioneer_p3dx(self) -> Pioneer_p3dx:
		children = self.get_objects_children('sim.handle_scene', children_type='sim.object_shape_type', filter_children=1 + 2)
		for h in children:
			name = self.get_object_name(h)
			if name == 'Pioneer_p3dx':
				return Pioneer_p3dx(self, h)

	def set_joint_target_velocity(self, handle, target):
		return self.client.simxSetJointTargetVelocity(handle, target, self.client.simxServiceCall())

	def pause(self):
		self.client.simxPauseSimulation(self.client.simxServiceCall())

# r2d2
# BallRobot
# OmniPlatform
