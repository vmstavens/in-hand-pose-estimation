#!/usr/bin/env python3

from typing import List
from sr_robot_commander.sr_hand_commander import SrHandCommander
import rospy
from gazebo_msgs.msg import ContactsState, ContactState
import time
from ros_utils_py import devprint
from shadow_hand.ContactPoint import ContactPoint

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
		
		self.__hand_commander: SrHandCommander = hc
		self.__update_freq: float = update_freq
		self.__finger_type: str = finger_type
		self.__chirality: str = chirality
		self.__tac_topic: str = f"/contacts/{chirality}_{finger_type}/distal"
		self.__contact_points: list[ContactPoint] = []
		self.__name: str = f"{self.__chirality}_{self.__finger_type.upper()}"

		# there are 4 joint, in range the last number is exclusive
		self.__joint_names = [ f"{self.__chirality}_{self.__finger_type.upper()}J{i}" for i in range(1,5) ] 
  
		# subscribe and print the found contacts
		self.__tac_sub = rospy.Subscriber(f"/contacts/{self.__chirality}_{self.__finger_type}/distal", ContactsState, callback = self.__read_biotac_sim, queue_size = 1)

	def get_name(self) -> str:
		"""get the finger's name

		Returns:
			str: finger's name
		"""
		return f"{self.__chirality}_{self.__finger_type.upper()}"

	def get_update_freq(self) -> float:
		"""get the tactile data update frequency in Hz

		Returns:
			float: tactile data update frequency
		"""
		return self.__update_freq

	def get_tac_type(self) -> str:
		"""get the tactile sensor type. Currently only two options are present: biotac_sim and None

		Returns:
			str: the tactile sensor type
		"""
		if self.__is_biotac_sim_live():
			return "biotac_sim"
		else:
			return "None"

	def __read_biotac_sim(self, data: ContactsState) -> None:
		"""the callback function used to sample tactile data from biotac_sim_plugin

		Args:
			data (ContactsState): a Gazebo ContactsState
		"""
		# sleep
		time.sleep( 1.0 / self.__update_freq )

		if self.__contact_has_been_made(data.states):
			
			# the data message only contains 1 ContactState i.e. data.states = ContactsStates = [ContactState]
			cs: ContactState = data.states[0]

			for i in range(len(cs.contact_positions)):
				# fill out contact points
				self.__contact_points.append(ContactPoint(contact_position = cs.contact_positions[i],contact_normal = cs.contact_normals[i], wrench = cs.wrenches[i], depth = cs.depths[i]))


	def get_tac_topic(self) -> str:
		"""get the topic this finger reads from to get its tactile data

		Returns:
			str: topic where the tactile data is found
		"""
		return self.__tac_topic

	def set_q(self, q: List[float]) -> bool:
		"""sets the configuration of the finger to q in the order: tip_q, middle_q and base_q

		Args:
			q (list): the q you want the finger to have

		Returns:
			bool: did the set function succeed
		"""
		q_dict = dict(zip(self.__joint_names, q))

		# try and set the finger q
		try:
			devprint(rospy.get_name() + f" attempting to set joint value {q}...")
			self.__hand_commander.move_to_joint_value_target(q_dict)
			devprint(rospy.get_name() + " succeeded...")
			return True
		except:
			rospy.logerr(f"failed to set joint value {q}...")
			return False


	def get_q(self) -> List[float]:
		"""get the current joint configuration

		Returns:
			List[float]: current joint configuration
		"""
		return self.__hand_commander.get_joints_position()[self.__name]

	def get_contact_points(self,timeout:float = 5.0) -> List[ContactPoint]:
		"""gets the current tactile data, or wait until something is available. The waiting time is determined by the argument timeout

		Returns:
			List[ContactPoint]: list of contact points
		"""

		start = rospy.get_rostime()

		while len(self.__contact_points) == 0:
	
			now = rospy.get_rostime()

			if (now.to_sec() - start.to_sec()) > timeout:
				rospy.logerr(f"Tactile data was not received within the timeout of {timeout}... retuning with an empty array")
				return []

			devprint("awaiting contact points...")
			time.sleep(1.0 / self.__update_freq)
			continue
		
		return self.__contact_points

	def __contact_has_been_made(self, cs: ContactsState) -> bool:
		"""check if tactile data has been received

		Args:
			cs (ContactsState): contact state from callback

		Returns:
			bool: success
		"""
		if len(cs) > 0:
			return True
		else:
			return False

	def __is_biotac_sim_live(self) -> bool:
		"""check if the biotac_sim_plugin package has been loaded

		Returns:
			bool: success
		"""
		published_topics = rospy.get_published_topics()
		does_the_topic_exist_list = [s[0] == self.__tac_topic for s in published_topics]
		result = any(does_the_topic_exist_list)
		if result:
			devprint("the biotac_sim_plugin topics have successfully been found....")
			return True
		else:
			return False
