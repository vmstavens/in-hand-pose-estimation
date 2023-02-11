#!/usr/bin/env python3

import rospkg
import rospy
from ros_utils_py.utils import keep_alive
from ros_utils_py.log import Logger
import time
from shadow_hand import ShadowHand
import math

def main() -> None:

	# waiting period for robot hand to start up...
	waiting_time: int = 5  # s
	rospy.loginfo(f"waiting {waiting_time} for hand to start...")
	time.sleep(waiting_time)

	# # joint configuration, from base to tip (does this make contact with the pen? yes)
	q: list = [0.0, 0.0, math.pi / 2.0]

	# create shadow hand object
	sh = ShadowHand()
 
	# set the index finger to q
	sh.index_finger.set_q(q)
 
	# init logger and directory
	log = Logger()
 
	# print tactile information when available
	while (True):
		time.sleep(1)
		if len(sh.index_finger.get_contact_points()) != 0:
			log.info("THIS IS A LOGGING TEST")
			log.info("contact point coordinates: \n" + sh.index_finger.get_contact_points()[0].contact_position.__str__())
		else:
			log.info("waiting for contacts...")

if __name__ == '__main__':
	try:
		rospy.init_node("finger_control_demo", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass