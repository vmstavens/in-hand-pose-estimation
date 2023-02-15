#!/usr/bin/env python3

from ros_utils_py.log import Logger
import time
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
from typing import List, Dict, Optional
import rospy
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.msg import ContactState

from shadow_hand.ShadowFinger import ShadowFinger
from shadow_hand.ShadowWrist import ShadowWrist

class ShadowHand:
	"""A wrapper class for interacting with the Shadow Dexterous Hand"""

	def __init__(self):
     
		# logger
		self.__logger = Logger()
     
		# waiting period for robot hand to start up...
		__waiting_time: int = 5  # s
		self.__logger.info(f"waiting {__waiting_time} for hand to start...")
		time.sleep(__waiting_time)

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

		# define wrist
		self.__wrist = ShadowWrist(hc=self.__hand_commander,chirality=self.__hand_chirality)

	@property
	def hand_chirality(self) -> str:
		"""gets the hand's chirality (e.g. lh or rh)"""
		return self.__hand_chirality

	@property
	def joints_names(self) -> List[str]:
		"""get the joints of the shadow hand"""
		return self.__joints

	@property
	def fingers(self) -> List[ShadowFinger]:
		"""gets the ShadowFingers for this ShadowHand"""
		return [self.thumb_finger, self.index_finger, self.middle_finger, self.ring_finger, self.little_finger]

	@property
	def q(self) -> Dict[str, float]:
		"""returns the angle for each joint in the hand in radians e.g. {'rh_FFJ1': 0.20, 'rh_FFJ2': 0.29, 'rh_FFJ3': 0.38, ... """
		return self.__hand_commander.get_joints_position()

	@property
	def dq(self) -> Dict[str, float]:
		"""returns the angular velocity for each joint in radians/s e.g. {'rh_FFJ1': 0.20, 'rh_FFJ2': 0.29, 'rh_FFJ3': 0.38, ... """
		return self.__hand_commander.get_joints_velocity()

	@property
	def tau(self) -> Dict[str,float]:
		"""returns the torque for each joint in Nm e.g. {'rh_FFJ1': 0.20, 'rh_FFJ2': 0.29, 'rh_FFJ3': 0.38, ... """
		return self.__hand_commander.get_joints_effort()

	@property
	def contact_states(self) -> Dict[str, Optional[ContactState]]:
		"""get a dictionary of all fingers' names and their contact states"""
		result = {
			self.thumb_finger.name  : self.thumb_finger.contact_state,
			self.index_finger.name  : self.index_finger.contact_state,
			self.middle_finger.name : self.middle_finger.contact_state,
			self.ring_finger.name   : self.ring_finger.contact_state,
			self.little_finger.name : self.little_finger.contact_state
		}
		return result

	@property
	def tac_type(self) -> List[str]:
		"""get a list of the tactile types of the hand as used on every finger in order: [thumb_finger, index_finger, middle_finger, ring_finger, little_finger]"""
		return [f.tac_type for f in self.fingers]

	@property
	def wrist(self) -> ShadowWrist:
		"""get the shadow hand's wrist object ShadowWrist"""
		return self.__wrist

	@property
	def is_in_contact(self) -> bool:
		return any(f.is_in_contact for f in self.fingers)

	def set_q(self,des_state: Dict[ShadowFinger,List]) -> bool:
		"""sets all joint configurations as specified in the dictionary. One such example can be seen below

			hand_q = {
				sh.thumb_finger  : [0.0, 0.0, π/2.0],
				sh.index_finger  : [0.0, 0.0, π/2.0],
				sh.middle_finger : [0.0, 0.0, π/2.0],
				sh.ring_finger   : [0.0, 0.0, π/2.0],
				sh.little_finger : [0.0, 0.0, π/2.0],
				sh.wrist         : [0.0, 0.0]
			}
			if you want certain joints unchanged, simply leave a None
		Returns:
			bool: if the setting succeeded
		"""
		# set the state given for all fingers, if one does not succeed, return false, else return true
		for f in des_state.keys():
			if not f.set_q( des_state[f] ):
				return False

		return True
		# joints_states = {'rh_FFJ1': 90, 'rh_FFJ2': 90, 'rh_FFJ3': 90, 'rh_FFJ4': 0.0,
	        #            'rh_MFJ1': 90, 'rh_MFJ2': 90, 'rh_MFJ3': 90, 'rh_MFJ4': 0.0,
	        #            'rh_RFJ1': 90, 'rh_RFJ2': 90, 'rh_RFJ3': 90, 'rh_RFJ4': 0.0,
	        #            'rh_LFJ1': 90, 'rh_LFJ2': 90, 'rh_LFJ3': 90, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
	        #            'rh_THJ1': 40, 'rh_THJ2': 35, 'rh_THJ3': 0.0, 'rh_THJ4': 65, 'rh_THJ5': 15,
	        #            'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

		# for f in self.get_fingers():
		# 	hand_dict[f.get_name()] = dict(zip(f.get_joint_names(), q_state[f.get_name()]))
   
		# self.__logger.info(hand_dict.__str__())

		# # try and set the fingers q
		# try:
		# 	self.__logger.warn(rospy.get_name() + f" attempting to set joint value {q_state}...")
		# 	self.__hand_commander.move_to_joint_value_target(hand_dict)
		# 	self.__logger.success(rospy.get_name() + " succeeded...")
		# 	return True
		# except:
		# 	rospy.logerr(f"failed to set joint value {q_state}...")
		# 	return False
