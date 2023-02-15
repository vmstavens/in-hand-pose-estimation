#!/usr/bin/env python3
import math

import rospy
from gazebo_msgs.msg import ContactsState
from ros_utils_py.log import Logger
from ros_utils_py.utils import keep_alive, kill_on_ctrl_c, create_pkg_dir
from ros_utils_py.plot import Plotter
from shadow_hand import ShadowHand
from random import random
from datetime import date
import rospkg
import json
import os

# init logger
log = Logger()

def main() -> None:
 
	# create shadow hand object
	sh : ShadowHand = ShadowHand()
 
	# joint configuration, from base to tip (does this make contact with the pen? yes)
	test_q: list = [0.0, 0.0, math.pi / 2.0]
 
	hand_q = {
		sh.index_finger  : test_q,
		sh.middle_finger : test_q,
		sh.ring_finger   : test_q,
		sh.little_finger : test_q
	}
 
	# set the hand q
	sh.set_q(hand_q)

	# create the data directory
	data_dir = create_pkg_dir(__file__, "data/")

	json_file = open(data_dir + date.today().strftime("%Y%m%d_%H%M%S") + ".json", "w")
	json_file.truncate(0)
	json_data = {}

	while True:
		rospy.sleep(1)
		if not sh.is_in_contact:
			log.warn("waiting for contact of any kind...")
			continue
		
		t = 0
		for f in sh.fingers:
			# data = {
			# 	""
			# }
			xs = [X.contact_position.x for X in cps]
			ys = [Y.contact_position.y for Y in cps]
			zs = [Z.contact_position.z for Z in cps]
			json_data[f.get_name()] = {"x": xs, "y": ys, "z": zs}
			log.info(json_data.__str__())
		json.dump(json_data, json_file, indent=4)
		break


if __name__ == '__main__':
	try:
		rospy.init_node("sr_tactile_perception", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
