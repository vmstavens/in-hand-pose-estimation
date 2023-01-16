#!/usr/bin/env python3

from __future__ import absolute_import
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
import time
from datetime import datetime
from ros_utils_py.utils import devprint
from enum import Enum, unique, auto

class Hand:
 

		
	class ENUM_FINGERS(Enum):
		def __init__(self, finger_names: list):
			rospy.logerr("---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
			rospy.logerr(isinstance(finger_names,list))
			if not isinstance(finger_names,list):
				ValueError("finger_names is not of type list[str]")
				return

			self.THUMB         = finger_names[0] # THUMB
			self.INDEX_FINGER  = finger_names[1] # INDEX_FINGER 
			self.MIDDLE_FINGER = finger_names[2] # MIDDLE_FINGER 
			self.RING_FINGER   = finger_names[3] # RING_FINGER 
			self.LITTLE_FINGER = finger_names[4] # LITTLE_FINGER
			
	def __init__(self):
		# waiting period for robot hand to start up...
		waiting_time : int = 5 # s
		devprint(f"waiting {waiting_time} for hand to start...")
		time.sleep(waiting_time)

		# hand finder - gets data from the found hand(s)
		self.__hand_finder : HandFinder = HandFinder()

		# hand config object
		self.__hand_parameters = self.__hand_finder.get_hand_parameters()
  
		# serial numbers for the hands found
		self.__hand_serial_numbers : list = list(self.__hand_parameters.mapping.keys())

		# the 0'th hand serial number (['1234', '0']) '0' is here a dummy and '1234' is the serial for the simulated hand
		self.__hand_serial : str = self.__hand_serial_numbers[0]
  
		# get a hand commander, which can communicate with the hand specified by the inputs
		self.__hand_commander = SrHandCommander(hand_parameters=self.__hand_parameters, hand_serial=self.__hand_serial)
  
		# in the hand config object we get the prefix (i.e. chirality) (either 'rh' or 'lh') depending on the type of hand with the serial number
		self.__hand_chirality = self.__hand_parameters.mapping[self.__hand_serial]

		# get all the joint names in the hand with the hand prefix found above (e.g. 'rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', ...)
		self.__joints = self.__hand_finder.get_hand_joints()[self.__hand_chirality]

		# joints=['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4', 'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4', 'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5', 'rh_WRJ1', 'rh_WRJ2', 'rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4', 'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4', 'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5', 'rh_WRJ1', 'rh_WRJ2']
	
		self.FINGERS = self.ENUM_FINGERS([
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

	def set_finger(self, finger:Enum, q:list) -> bool:
		"""sets the finger (FINGER) configuration to the one provided as q [base_joint, middle_joint, tip_joint]

		Args:
			FINGER (Enum): One of the five fingers: THUMB, INDEX_FINGER, MIDDLE_FINGER, RING_FINGER or LITTLE_FINGER
			q (list): a list of the joint values you want in radians

		Returns:
			bool: did the function successfully complete
		"""
		joints = [ f"{finger.value}J{j}" for j in range(1,4) ]
		devprint(f"these are my joints {joints} in finger {finger}")
		des_q = dict(zip(joints, q))
		try:
			devprint(f"attempting to set joint value {q}...")
			self.__hand_commander.move_to_joint_value_target(des_q)
			return True
		except:
			devprint(f"failed to set joint value {q}...")
			return False


# http://wiki.ros.org/PyStyleGuide
def testing() -> None:
	while(True):
		time.sleep(1)
		print("testing")


def main() -> None:

	test_q = [0.35, 0.18, 0.38]
	hand : Hand = Hand()
 
	if hand.set_finger(hand.FINGERS.INDEX_FINGER, test_q):
		devprint("GREAT SUCCESS")
		testing()
	else:
		devprint("FAILED")
		exit()

	# time.sleep(5)
	
	# # hand finder - gets data from the found hand(s)
	# hand_finder = HandFinder()
	
	# # hand config object
	# hand_parameters = hand_finder.get_hand_parameters()
	
	# # serial numbers for the hands found
	# hand_serial_numbers = list(hand_parameters.mapping.keys())
	
	# # the 0'th hand serial number (['1234', '0']) '0' is here a dummy and '1234' is the serial for the simulated hand
	# hand_serial = hand_serial_numbers[0]

	# # get a hand commander, which can communicate with the hand specified by the inputs
	# hand_commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial)

	# # in the hand config object we get the mapping (either 'rh' or 'lh') depending on the type of hand with the serial number
	# hand_mapping = hand_parameters.mapping[hand_serial]

	# # get all the joint names in the hand with the hand mapping found above (e.g. 'rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', ...)
	# joints = hand_finder.get_hand_joints()[hand_mapping]

	# # create three angular values in radians
	# position_values = [0.35, 0.18, 0.38]

	# # Moving to a target determined by the values in position_values.
	# devprint("Hand moving to script specified target")
	# position_1 = dict(zip(joints, position_values))
	# devprint(f"{hand_parameters=} |\n {hand_serial_numbers=} |\n {hand_serial=} |\n {hand_commander=} |\n {hand_mapping=} |\n {joints=} |\n {position_values=} |\n {position_1=} |\n ")
	# hand_commander.move_to_joint_value_target(position_1)

	# named_target_1 = "pack"
	# devprint("Hand moving to named target: " + named_target_1)
	# hand_commander.move_to_named_target(named_target_1)

	# named_target_2 = "open"
	# devprint("Hand moving to named target: " + named_target_2)
	# hand_commander.move_to_named_target(named_target_2)

	# # Hand joints state, velocity and effort are read and displayed to screen.
	# hand_joints_state = hand_commander.get_joints_position()
	# hand_joints_velocity = hand_commander.get_joints_velocity()
	# hand_joints_effort = hand_commander.get_joints_effort()

	# devprint("Hand joints position \n " + str(hand_joints_state) + "\n")
	# devprint("Hand joints velocity \n " + str(hand_joints_velocity) + "\n")
	# devprint("Hand joints effort \n " + str(hand_joints_effort) + "\n")

	# # Tactile type and state are read and displayed to screen.
	# tactile_type = hand_commander.get_tactile_type()
	# tactile_state = hand_commander.get_tactile_state()
	# devprint("Tactile type \n " + str(tactile_type) + "\n")
	# devprint("Tactile state \n " + str(tactile_state) + "\n")

	testing()

if __name__ == '__main__':
	try:
		rospy.init_node("test", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
