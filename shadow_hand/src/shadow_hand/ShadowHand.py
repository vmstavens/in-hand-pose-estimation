#!/usr/bin/env python3

from ros_utils_py.utils import devprint
import time
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
from typing import List, Dict
import rospy
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.msg import ContactState

from shadow_hand.ShadowFinger import ShadowFinger
from shadow_hand.ContactPoint import ContactPoint

class ShadowHand:
	"""A wrapper class for interacting with the Shadow Dexterous Hand"""

	def __init__(self):
		# waiting period for robot hand to start up...
		waiting_time: int = 5  # s
		devprint(f"waiting {waiting_time} for hand to start...")
		time.sleep(waiting_time)

		# hand finder - gets data from the found hand(s)
		self.__hand_finder: HandFinder = HandFinder()

		# hand config object
		self.__hand_parameters = self.__hand_finder.get_hand_parameters()

		# serial numbers for the hands found
		self.__hand_serial_numbers: list = list(
			self.__hand_parameters.mapping.keys())

		# the 0'th hand serial number (['1234', '0']) '0' is here a dummy and '1234' is the serial for the simulated hand
		self.__hand_serial: str = self.__hand_serial_numbers[0]

		# get a hand commander, which can communicate with the hand specified by the inputs
		self.__hand_commander = SrHandCommander(
			hand_parameters=self.__hand_parameters, hand_serial=self.__hand_serial)

		# in the hand config object we get the prefix (i.e. chirality) (either 'rh' or 'lh') depending on the type of hand with the serial number
		self.__hand_chirality = self.__hand_parameters.mapping[self.__hand_serial]

		# get all the joint names in the hand with the hand prefix found above (e.g. 'rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', ...)
		self.__joints = self.__hand_finder.get_hand_joints()[self.__hand_chirality]

		# define fingers
		self.thumb_finger  = ShadowFinger(finger_type=ShadowFinger.FINGERS_NAMES.THUMB_FINGER,  hc=self.__hand_commander)
		self.index_finger  = ShadowFinger(finger_type=ShadowFinger.FINGERS_NAMES.INDEX_FINGER,  hc=self.__hand_commander)
		self.middle_finger = ShadowFinger(finger_type=ShadowFinger.FINGERS_NAMES.MIDDLE_FINGER, hc=self.__hand_commander)
		self.ring_finger   = ShadowFinger(finger_type=ShadowFinger.FINGERS_NAMES.RING_FINGER,   hc=self.__hand_commander)
		self.little_finger = ShadowFinger(finger_type=ShadowFinger.FINGERS_NAMES.LITTLE_FINGER, hc=self.__hand_commander)

	def get_hand_chirality(self) -> str:
		"""gets the hand's chirality (e.g. lh or rh)

		Returns:
			str: the hand's chirality
		"""
		return self.__hand_chirality


	def get_joints(self):
		"""get the joints of the shadow hand

		Returns:
			list[str]: a list of joint names
		"""
		return self.__joints

	def get_fingers(self) -> List[ShadowFinger]:
		"""gets the ShadowFingers for this ShadowHand

		Returns:
			List[ShadowFinger]: ShadowFingers
		"""
		return [self.thumb_finger, self.index_finger, self.middle_finger, self.ring_finger, self.little_finger]

	def get_q(self) -> Dict[str, float]:
		"""returns the angle for each joint in radians

		Returns:
			Dict[str,int]: each joint and their angle organized in a dictionary e.g. {'rh_FFJ1': 0.20, 'rh_FFJ2': 0.29, 'rh_FFJ3': 0.38, ...
		"""
		return self.__hand_commander.get_joints_position()

	def get_dq(self) -> Dict[str, float]:
		"""returns the angular velocity for each joint in radians/s

		Returns:
			Dict[str,int]: each joint and their angular velocity organized in a dictionary e.g. {'rh_FFJ1': 0.20, 'rh_FFJ2': 0.29, 'rh_FFJ3': 0.38, ...
		"""
		return self.__hand_commander.get_joints_velocity()

	def get_tau(self) -> Dict[str,float]:
		"""returns the torque for each joint in Nm

		Returns:
			Dict[str,int]: each joint and their torque organized in a dictionary e.g. {'rh_FFJ1': 0.20, 'rh_FFJ2': 0.29, 'rh_FFJ3': 0.38, ...
		"""
		return self.__hand_commander.get_joints_effort()

	def get_contact_points(self) -> Dict[str,List[ContactPoint]]:
		"""get a dictionary of all fingers' names and their contact points

		Returns:
			Dict[str,List[ContactPoint]]: dictionary of all fingers' names and their contact points
		"""
		result = {
			self.thumb_finger.get_name()  : self.thumb_finger.get_contact_points(),
			self.index_finger.get_name()  : self.index_finger.get_contact_points(),
			self.middle_finger.get_name() : self.middle_finger.get_contact_points(),
			self.ring_finger.get_name()   : self.ring_finger.get_contact_points(),
			self.little_finger.get_name() : self.little_finger.get_contact_points()
		}
		return result

	def get_tac_type(self) -> str:
		"""get the tactile type of the hand as used on every finger

		Returns:
			str: the tactile sensor type
		"""
		return self.index_finger.get_tac_type()

