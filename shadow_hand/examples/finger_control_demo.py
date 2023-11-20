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
from shadow_hand import ShadowHand
import math as m

def main() -> None:

	# TODO : 
	# 	1) perform planned motion
	# 	2) perform planned motion + gravity
	# 	3) place two boxes into world
	# 	4) build plan to pick up box 1
	# 	5) build plan to place box 1 onto box 2
	# 	6) complete
	#   ----- pain threshold -----
	# 	7) attempt to find digit tactile plugin and mesh for gazebo.  

	# waiting period for robot hand to start up...
	waiting_time: int = 10  # s
	rospy.loginfo(f"waiting {waiting_time} for hand to start...")
	time.sleep(waiting_time)

	# joint configuration, from base to tip (does this make contact with the pen? yes)
	q: list = [m.pi/4.0, m.pi/4.0, 0.0]
	# q: list = [0.0, m.pi / 2.0, m.pi/2.0]
	
	# q: list = [0.0, m.pi / 2.0, 0.0]

	# create shadow hand object
	sh = ShadowHand()
 
	# set the index finger to q
	sh.index_finger.set_q(q)
 
	# init logger and directory
	log = Logger()
	# 1.1751135385968023, 1.1811629668337202
 
	# print tactile information when available
	while (True):
		time.sleep(1)
		log.warn(str(sh.index_finger.q))
		
		if sh.index_finger.is_in_contact:
			log.info(f"contact point coordinates: {sh.index_finger.contact_state}")
		else:
			log.info("waiting for contacts...")
	keep_alive(rospy.get_name())

if __name__ == '__main__':
	try:
		rospy.init_node("finger_control_demo", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass