#!/usr/bin/env python
import rospy, pickle, time
from utils.robot import *
from utils.constants import *

from geometry_msgs.msg import Pose
import numpy as np
import PyKDL
import multiprocessing
import tfx
import datetime
import time

import cv_bridge
import cv2
from sensor_msgs.msg import Image, CameraInfo

class Environment:
	def __init__(self, env_time, rewarding_rules = None):
		"""initialize an environment with 
			1. robot right arm(so far)

			also with the following parameters:
			1. env_time: a goal time interval that this environment exists
			2. time_counter: to keep on track with current time.
		"""
		self.psmR = robot("PSM1")
		self.psmL = robot("PSM2")
		self.env_time = env_time
		self.time_counter = 0
		self.rewarding_rules = rewarding_rules
		self.state = self.reset(self.psmR) # For now
		self.done = False
		# TODO: implement self.action_space and self.observation_space that defined as
		# shape of action_space and shape of observation_space


	def reset(self, arm):
		"""get back to default position and return the default position as observation"""
		"""reset for left arm is still undone. Parallel thing need to be fixed;"""
		rot = tfx.tb_angles(INIT_POS[2][0], INIT_POS[2][1], INIT_POS[2][2])
		arm.move_cartesian_frame(tfx.pose(INIT_POS[1], rot))
		arm.close_gripper()
		self.time_counter = 0
		return {self.psmR: [self.psmR.get_current_cartesian_position(), False], self.psmL: [self.psmL.get_current_cartesian_position(), False]}
		# TODO: reset apply to the platform and camera also

	def step(self, arm, action):
		"""it now takes in a list/tuple of [[position, rotation], gripper_open], execute the action(move arm to indicated pos, rot, gripper_open)
		and returns:"""
		"""
		observation: object --- an environment-specific object representing your observation of the environment; 
			in this env new list of pos, rot
		reward: float --- amount of reward achieved by the previous action. 
			The scale varies between environments, and can also be customized using BLANK method.
		done: boolean --- whether it's time to reset the environment again. 
			Most tasks are divided up into well-defined episodes, and done being True indicates the episode has terminated. 
		info: dict --- value TBD, naive thoughts are to include info for next step
		"""
		# Start Timing for this stepping action
		start_time = time.time()
		# TODO: make more complicating actions
		pos, rot = action[0]
		gripper_open = action[1]
		t1, t2, t3 = pos
		if not self.done:
			# observation = action
			# TODO: check if out of range
			# Take action
			obs_pos = arm.get_current_cartesian_position().position
			obs_rot = tfx.tb_angles(arm.get_current_cartesian_position().rotation)
			for i in range(0, 3):
				pos[i] += obs_pos[i]
			r1 = obs_rot.yaw_rad
			r2 = obs_rot.pitch_rad
			r3 = obs_rot.roll_rad
			new_rot = tfx.tb_angles(r1 + rot[0], r2 + rot[1], r3 +rot[2], rad = True)
			
			arm.move_cartesian_frame(tfx.pose(pos, new_rot))
			if gripper_open:
				arm.open_gripper(CUT_OPEN_ANGLE)
				time.sleep(CUT_OPEN_TIME)
			else:
				arm.close_gripper()
			observation = [arm.get_current_cartesian_position(), gripper_open]
		self.state[arm] = observation
		# Get reward
		reward = self.reward(self.state, action)
		# Determine done
		self.done = self.time_counter >= self.env_time
		# TODO: implement info
		info = {}
		# end timing for current stepping
		self.time_counter += (time.time() - start_time)
		return [observation, reward, self.done, info]

	# def step_get_info(self):
	# 	observation = arm.get_current_cartesian_position()
	# 	self.state = [self.psmR.get_current_cartesian_position(), self.psmL.get_current_cartesian_position()]
	# 	# Get reward
	# 	reward = self.reward(self.state, action)
	# 	# Determine done
	# 	done = self.time_counter >= self.env_time
	# 	# TODO: implement info
	# 	info = {}
	# 	# end timing for current stepping
	# 	self.time_counter += (time.time() - start_time)
	# 	return [observation, reward, done, info]

	def reward(self, state = None, action = None):
		"""we define rewarding rules as a customized function that maps (state, action) pair to float."""
		if self.rewarding_rules:
			return self.rewarding_rules(state, action)
		else:
			return 0

# def parallel_step(env, actionR, actionL):
# 	p = multiprocessing.Pool(2)
# 	return p.map(step_wrapper, [(env, "r", actionR), (env, "l", actionL)])

# def step_wrapper(env, arm, action):
# 	if arm == "r":
# 		return env.step(env.psmR, action)
# 	elif arm == "l":
# 		return env.step(env.psmL, action)

# test = Environment(10)


# if __name__ == '__main__':
# 	parallel_step(test, [[0.001, 0.001, 0.001], [0.03, 0.03, 0.03]], [[0.001, 0.001, 0.001], [0.03, 0.03, 0.03]])