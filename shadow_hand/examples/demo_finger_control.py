#!/usr/bin/env python3

import rospy
from ros_utils_py.utils import devprint, keep_alive
from time import time
from shadow_hand import ShadowHand

def main() -> None:

	# joint configuration, from base to tip
	test_q = [0.35, 0.18, 0.38]

	# hand object
	hand: ShadowHand = ShadowHand()

	# set hand finger to test_q
	hand.set_finger(hand.FINGERS.INDEX_FINGER, test_q)
	
	devprint(str(hand.get_q()))
 
	keep_alive(rospy.get_name())

if __name__ == '__main__':
	try:
		rospy.init_node("example_shadow_hand", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass