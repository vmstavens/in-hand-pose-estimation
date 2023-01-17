#!/usr/bin/env python3

from ros_utils_py.utils import devprint
import time
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
from typing import List, Dict


class ShadowHand:

	HOME_POSITION = [0.0, 0.0, 0.0]

	class ENUM_FINGERS():
		def __init__(self, finger_names: list):
			if not isinstance(finger_names, list):
				ValueError("finger_names is not of type list[str]")
				return

			self.THUMB = finger_names[0]          # THUMB
			self.INDEX_FINGER = finger_names[1]   # INDEX_FINGER
			self.MIDDLE_FINGER = finger_names[2]  # MIDDLE_FINGER
			self.RING_FINGER = finger_names[3]    # RING_FINGER
			self.LITTLE_FINGER = finger_names[4]  # LITTLE_FINGER

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

		# joints=['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4', 'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4', 'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5', 'rh_WRJ1', 'rh_WRJ2', 'rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4', 'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4', 'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5', 'rh_WRJ1', 'rh_WRJ2']

		self.FINGERS = self.ENUM_FINGERS(finger_names=[
			f"{self.__hand_chirality}_TH",
			f"{self.__hand_chirality}_FF",
			f"{self.__hand_chirality}_MF",
			f"{self.__hand_chirality}_RF",
			f"{self.__hand_chirality}_LF"
		])

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

	def set_finger(self, finger: ENUM_FINGERS, q: List[float]) -> bool:
		"""sets the finger (FINGER) configuration to the one provided as q [base_joint, middle_joint, tip_joint]

		Args:
			FINGER (ENUM_FINGERS): One of the five fingers: THUMB, INDEX_FINGER, MIDDLE_FINGER, RING_FINGER or LITTLE_FINGER
			q (list): a list of the joint values you want in radians

		Returns:
			bool: did the function successfully complete
		"""
		joints = [f"{finger}J{j}" for j in range(1, 4)]
		devprint(f"these are my joints {joints} in finger {finger}")
		des_q = dict(zip(joints, q))
		try:
			devprint(f"attempting to set joint value {q}...")
			self.__hand_commander.move_to_joint_value_target(des_q)
			return True
		except:
			devprint(f"failed to set joint value {q}...")
			return False

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
 
	def get_tac_type(self) -> str: # TODO : fix
		return self.__hand_commander.get_tactile_type()

	def get_tac_pressure(self) -> Dict[str, float]:  # TODO : fix
		return self.__hand_commander.get_tactile_state()

# how to get tactile information? gazebo pressure?