#!/usr/bin/env python3

import rospy
from ros_utils_py.utils import devprint, keep_alive
import time
from shadow_hand import ShadowHand
import math
from gazebo_msgs.msg import ContactsState
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

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
	
	# print tactile information when available
	while (True):
		time.sleep(1)
		if len(sh.index_finger.get_contact_points()) != 0:
			devprint("\n" + sh.index_finger.get_contact_points()[0].contact_position.__str__())
		else:
			devprint("waiting for contacts...")

if __name__ == '__main__':
	try:
		rospy.init_node("finger_control_demo", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass