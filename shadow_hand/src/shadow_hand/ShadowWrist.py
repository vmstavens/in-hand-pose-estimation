#!/usr/bin/env python3

from typing import List, Optional, Dict, Union
from ros_utils_py.log import Logger
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from shadow_hand import ShadowFinger, ShadowHand

class ShadowWrist():
	def __init__(self, hc: SrHandCommander, chirality: str = "rh"):

		self.__log = Logger()
  
		self.__chirality = chirality
		self.__hand_commander = hc
		self.__number_of_joints = 2
		self.__joint_names = ["rh_WRJ1", "rh_WRJ2"]
  
	@property
	def chirality(self):
		"""gets the hand's chirality (e.g. lh or rh)"""
		return self.__chirality
  
	@property
	def name(self) -> str:
		"""get the wrist's name"""
		return f"{self.__chirality}_WR"

	@property
	def joint_names(self) -> List[str]:
		"""the joint names in a list i.e. ["rh_WRJ1", "rh_WRJ2"]"""
		return [f"{self.__chirality}_WRJ1", f"{self.__chirality}_WRJ2"]

	@property
	def number_of_joints(self) -> int:
		"""the number of joints, which for a ShadowWrist is 2"""
		return self.__number_of_joints

	@property
	def q(self) -> List[float]:
		"""get the current joint configuration"""
		return [self.__hand_commander.get_joints_position()[jn] for jn in self.joint_names]

	def set_q(self, q: List[Optional[float]]) -> bool:
		"""sets the configuration of the wrist to q in the order: yaw_q (waving motion), pitch_q (padding motion)
		"The Yaw rotation is more proximal, and has a range of -28º (towards little finger) to 10º (towards thumb), and a pulley diameter of 65mm."
		"The Pitch rotation is more distal, and has a range of -40º (extension) to 28º (flexion), and a pulley diameter of 31mm."

		Args:
			q (list): the q you want the wrist to have

		Returns:
			bool: did the set function succeed
		"""

		valid_q = self.make_valid_q(q)

		# create the joint dictionary
		q_dict = dict(zip(self.__joint_names, valid_q))

		# check if the dictionary has any None, if they do, replace it with the joint's current value
		for key in q_dict.keys():
			if q_dict[key] is None:
				q_dict[key] = self.q[self.joint_names.index(key)]

		try:
			self.__log.warn(rospy.get_name() + f" attempting to set {self.name} joint values {valid_q}...")
			self.__hand_commander.move_to_joint_value_target(q_dict)
			self.__log.success(rospy.get_name() + " succeeded...")
			return True
		except:
			self.__log.error(f"failed to set {self.name} joint values {q}...")
			return False

	
	def make_valid_q(self, q: List[Optional[float]]) -> List[float]:
		"""this function converts a q into a des_q which has the correct number of elements and constants joint values for Nones in q e.g.
		make_valid_q( [0.0, None, 0.0, pi] ) -> des_q = [0.0, current_q, 0.0, pi, current_q]
		"""

		valid_q = [qi for qi in q]

		# in this case the input is just not valid, and throw an error
		if len(q) > self.number_of_joints:
			raise ValueError(f"You cannot set more joints than what the finger has. Finger: {self.__name}, num of joints: {self.number_of_joints}, length of given q: {len(q)}")

		# if a q is given with a shorter length than number_of_joints, pad q with the joint's current values
		if len(q) < self.number_of_joints:
			# expand valid_q with self.q values
			d_length = self.number_of_joints - len(q)
			for i in range(d_length):
				valid_q.append(self.q[len(valid_q)])

		# overwrite None values in valid_q with self.q values
		for i, vq in enumerate(valid_q):
			valid_q[i] = vq if vq is not None else self.q[i]

		return valid_q
		
		# valid_q = self.make_valid_q(q)

		# # create the joint dictionary
		# q_dict = dict(zip(self.__joint_names, valid_q))

		# # check if the dictionary has any None, if they do, replace it with the joint's current value
		# for key in q_dict.keys():
		# 	if q_dict[key] is None:
		# 		q_dict[key] = self.q[self.joint_names.index(key)]

		# # in this case the input is just not valid, and throw an error
		# if len(q) > self.number_of_joints:
		# 	raise ValueError(f"You cannot set more joints than what the finger has. Finger: {self.__chirality}, num of joints: {self.number_of_joints}, length of given q: {len(q)}")

		# # if a q is given with a shorter length than number_of_joints, pad q with the joint's current values
		# elif len(q) < self.number_of_joints:

		# 	# overwrite None values in valid_q with self.q values
		# 	for i, qi in enumerate(q):
		# 		valid_q.append(qi if qi is not None else self.q[i])

		# 	# expand valid_q with self.q values
		# 	d_length = self.number_of_joints - len(q)

		# 	for i in range(d_length):
		# 		valid_q.append(self.q[len(q) + i - 1])
		# 	return valid_q
		# else:
		# 	return q