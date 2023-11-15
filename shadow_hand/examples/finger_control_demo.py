#!/usr/bin/env python3

import rospkg
import rospy
from ros_utils_py.utils import keep_alive
from ros_utils_py.log import Logger
import time
import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.arm_finder import ArmFinder
from sr_utilities.hand_finder import HandFinder
import math as m

def main() -> None:

	# waiting period for robot hand to start up...
	waiting_time: int = 10  # s
	rospy.loginfo(f"waiting {waiting_time} for hand to start...")
	time.sleep(waiting_time)

	
	# rospy.init_node("robot_commander_examples", anonymous=True)
	arm_commander = SrArmCommander(name="right_arm", set_ground=True)
	hand_finder = HandFinder()
	arm_finder = ArmFinder()

	# hand finder - gets data from the found hand(s)
	hand_finder: HandFinder = HandFinder()

	# hand config object
	hand_parameters = hand_finder.get_hand_parameters()

	# serial numbers for the hands found
	hand_serial_numbers: list = list(
		hand_parameters.mapping.keys())

	# the 0'th hand serial number (['1234', '0']) '0' is here a dummy and '1234' is the serial for the simulated hand
	hand_serial: str = hand_serial_numbers[0]

	# get a hand commander, which can communicate with the hand specified by the inputs
	hand_commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial)

	# To get the prefix or mapping of the arm joints. Mapping is the same as prefix but without underscore.
	arm_finder.get_arm_parameters().joint_prefix.values()
	arm_finder.get_arm_parameters().mapping.values()
	reference_frame = arm_commander.get_pose_reference_frame()
	rospy.logerr(reference_frame + " #######################################################################")
	arm_finder.get_arm_joints()
	
	new_pose = [0.5, 0.3, 1.2, 0, 1.57, 0]

	# To only plan
	arm_commander.plan_to_pose_target(new_pose)

	# To plan and move
	arm_commander.move_to_pose_target(new_pose)


	rospy.logerr(hand_commander.get_current_state())

	hand_commander.plan_to_named_target("open")
	hand_commander.move_to_named_target("open")
	hand_commander.plan_to_named_target("pack")
	hand_commander.move_to_named_target("pack")
	# # joint configuration, from base to tip (does this make contact with the pen? yes)
	# q: list = [0.0, m.pi, m.pi/2.0]
	
	# # q: list = [0.0, m.pi / 2.0, 0.0]

	# # create shadow hand object
	# sh = ShadowHand()
 
	# # set the index finger to q
	# sh.index_finger.set_q(q)
 
	# # init logger and directory
	# log = Logger()
 
 
	# # print tactile information when available
	# while (True):
	# 	time.sleep(1)
	# 	log.warn(str(sh.index_finger.q))
		
	# 	if sh.index_finger.is_in_contact:
	# 		log.info(f"contact point coordinates: {sh.index_finger.contact_state}")
	# 	else:
	# 		log.info("waiting for contacts...")
	# keep_alive(rospy.get_name())

if __name__ == '__main__':
	try:
		rospy.init_node("finger_control_demo", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass