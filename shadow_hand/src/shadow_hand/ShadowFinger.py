#!/usr/bin/env python3

from typing import List, Optional
from sr_robot_commander.sr_hand_commander import SrHandCommander
import rospy
from gazebo_msgs.msg import ContactsState, ContactState
import time
from ros_utils_py.log import Logger

class ShadowFinger:
	"""A wrapper class for interacting with the individual fingers of a Shadow Dexterous Hand"""

	class FINGERS_NAMES():
		"""Enum for shorthand finger strings"""
		THUMB_FINGER  = "th"
		INDEX_FINGER  = "ff"
		MIDDLE_FINGER = "mf"
		RING_FINGER   = "rf"
		LITTLE_FINGER = "lf"

	def __init__(self, finger_type: str, hc: SrHandCommander, update_freq: float = 2.0, chirality: str = "rh"):
		
		# logger
		self.__logger = Logger()
  
		self.__hand_commander: SrHandCommander = hc
		self.__update_freq: float = update_freq
		self.__finger_type: str = finger_type
		self.__chirality: str = chirality
		self.__tac_topic: str = f"/contacts/{chirality}_{finger_type}/distal"
		self.__contact_state: Optional[ContactState]
		self.__name: str = f"{self.__chirality}_{self.__finger_type.upper()}"

		# the label of each finger and how many joints they each have
		self.__fingers_and_num_of_joints = {"FF": 4, "MF": 4, "RF": 4, "LF": 5, "TH": 5}
  
		self.__num_of_joints = self.__fingers_and_num_of_joints[self.__finger_type.upper()]

		# there are 4 joint, in range the last number is exclusive
		self.__joint_names = [ f"{self.__chirality}_{self.__finger_type.upper()}J{i}" for i in range(1,self.__num_of_joints) ] 
  
		# subscribe and print the found contacts
		self.__tac_sub = rospy.Subscriber(f"/contacts/{self.__chirality}_{self.__finger_type}/distal", ContactsState, callback = self.__read_biotac_sim, queue_size = 1)

	@property
	def name(self) -> str:
		"""finger's name"""
		return f"{self.__chirality}_{self.__finger_type.upper()}"

	@property
	def joint_names(self) -> List[str]:
		"""joint names"""
		return self.__joint_names

	@property
	def number_of_joints(self) -> int:
		"""gets the number of joints for this particular finger"""
		return self.__num_of_joints

	@property
	def update_freq(self) -> float:
		"""get the tactile data update frequency in Hz"""
		return self.__update_freq

	@property
	def tac_type(self) -> str:
		"""tactile sensor type. Two options are present: biotac_sim and None"""
		return "biotac_sim" if self.__is_biotac_sim_live() else "None"

	@property
	def tac_topic(self) -> str:
		"""topic this finger reads from to get its tactile data"""
		return self.__tac_topic

	@property
	def q(self) -> List[float]:
		"""get the current joint configuration"""
		return self.__hand_commander.get_joints_position()[self.__name]

	@property
	def contact_state(self, timeout: float = 5.0) -> Optional[ContactState]:
		"""current tactile data, it returns empty i.e. [] if no contact is made"""
		return self.__contact_state

	@property
	def is_in_contact(self) -> bool:
		"""returns if the finger is in contact"""
		return False if self.contact_state is None else True

	def set_q(self, q: List[Optional[float]]) -> bool:
		"""the finger's q is set in the order: tip_q, q2, ... and base_q. If a None is given for a particular angle, the angle remains constant.
		Returns:
			bool: did the set function succeed
		"""
		# check if the input is legal
		if len(q) > self.number_of_joints:
			raise ValueError(f"You cannot set more joints than what the finger has. Finger: {self.__chirality}, num of joints: {self.number_of_joints}, length of given q: {len(q)}")

		# if a q is given with a shorter length than number_of_joints, pad q with the joint's current values
		if len(q) < self.number_of_joints:
			d_length = self.number_of_joints - len(q)
			for i in range(d_length):
				q.append(self.q[len(q) + i])
  
		# create the joint dictionary
		q_dict = dict(zip(self.__joint_names, q))
  
		# check if the dictionary has any None, if they do, replace it with the joint's current value
		for key in q_dict.keys():
			if q_dict[key] is None:
				q_dict[key] = self.q[self.joint_names.index(key)]
  
		try:
			self.__logger.warn(rospy.get_name() + f" attempting to set {self.name} joint values {q}...")
			self.__hand_commander.move_to_joint_value_target(q_dict)
			self.__logger.success(rospy.get_name() + " succeeded...")
			return True
		except:
			self.__logger.error(f"failed to set {self.name} joint values {q}...")
			return False

	def wait_for_contact(self, timeout: float = 5.0) -> Optional[ContactState]:
		"""gets the current tactile data, or wait until something is available. The waiting time is determined by the timeout argument

		Returns:
			ContactState | None: list of contact points or None
		"""
		start = rospy.get_rostime()

		# while None and an empty array is received, keep looking
		while self.contact_state is None:

			now = rospy.get_rostime()

			if (now.to_sec() - start.to_sec()) > timeout:
				rospy.logerr(f"Tactile data was not received within the timeout of {timeout}... retuning with an empty array")
				return None

			self.__logger.warn("awaiting contact points...")
			time.sleep(1.0 / self.__update_freq)
			continue

		return self.contact_state

	def __is_biotac_sim_live(self) -> bool:
		"""check if the biotac_sim_plugin package has been loaded

		Returns:
			bool: success
		"""
		published_topics = rospy.get_published_topics()
		does_the_topic_exist_list = [s[0] == self.__tac_topic for s in published_topics]
		result = any(does_the_topic_exist_list)
		if result:
			self.__logger.success("the biotac_sim_plugin topics have successfully been found....")
			return True
		else:
			self.__logger.error("the biotac_sim_plugin topics have NOT been found....")
			return False

	def __read_biotac_sim(self, data: Optional[ContactsState]) -> None:
		"""the callback function used to sample tactile data from biotac_sim_plugin

		Args:
			data (ContactsState): a Gazebo ContactsState, which is a list of contact states, each finger has a contact state with multiple points
		"""
		# sleep
		time.sleep( 1.0 / self.__update_freq )
  
		# if no data is received yet
		if data is None:
			self.__logger.warn("No tactile data has been received yet...")
			return None
			
		# ignore this error, as it is handled in the previous if statement
		self.contact_state = data.states if data is not None else None
