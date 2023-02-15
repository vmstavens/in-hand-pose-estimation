#!/usr/bin/env python3

from typing import List
from ros_utils_py.log import Logger
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

class ShadowWrist():
	def __init__(self, hc: SrHandCommander, chirality: str = "rh", update_freq: float = 2.0):

		self.__logger = Logger()
  
		self.__chirality = chirality
		self.__hand_commander = hc
  
	def get_chirality(self):
		"""get the wrist's chirality

		Returns:
			str: the wrist's chirality
		"""
		return self.__chirality
  
	def get_name(self) -> str:
		"""get the finger's name

		Returns:
			str: finger's name
		"""
		return f"{self.get_chirality()}_WR"

	def get_joint_names(self) -> List[str]:
		"""return the joint names in a list i.e. ["rh_WRJ1", "rh_WRJ2"]

		Returns:
			List[str]: joint names in a list
		"""
		return [f"{self.get_chirality()}_WRJ1", f"{self.get_chirality()}_WRJ2"]

	def set_q(self, q: List[float]) -> bool:
		"""sets the configuration of the wrist to q in the order: yaw_q (waving motion), pitch_q (padding motion)

		Args:
			q (list): the q you want the wrist to have

		Returns:
			bool: did the set function succeed
		"""
		q_dict = dict(zip(self.get_joint_names(), q))
		if len(q) != 2:
			raise ValueError(f"Wrong number of qs in set_q for wrist, expected len(q) = 2, got len(q) = {len(q)}")
		# try and set the wrist q
		try:
			self.__logger.warn(
				rospy.get_name() + f" attempting to set {self.get_name()} joint values {q}...")
			self.__hand_commander.move_to_joint_value_target(q_dict)
			self.__logger.success(rospy.get_name() + " succeeded...")
			return True
		except:
			rospy.logerr(f"failed to set {self.get_name()} joint values {q}...")
			return False
		