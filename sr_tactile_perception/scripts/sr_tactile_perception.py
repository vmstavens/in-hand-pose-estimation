#!/usr/bin/env python3
import math

import rospy
from ros_utils_py.log import Logger
from ros_utils_py.utils import create_pkg_dir, keep_alive, kill_on_ctrl_c
from shadow_hand import ShadowHand
from random import random
from datetime import date
import json
from typing import Dict, List
from geometry_msgs.msg import Vector3
from ros_utils_py.msg import PointCloud
from ros_utils_py.geometry import geometry

# init logger
log = Logger()


def main() -> None:

	pub = rospy.Publisher("live_plotter", PointCloud, queue_size=1)

	# create shadow hand object
	sh: ShadowHand = ShadowHand()

	# joint configuration, from base to tip (does this make contact with the pen? yes)
	test_q: list = [math.pi / 8.0, 0.0, math.pi / 2.0]

	hand_q = {
		sh.index_finger: test_q,
		sh.middle_finger: test_q,
		sh.ring_finger: test_q,
		sh.pinky_finger: test_q,
	}

	# set the hand q
	sh.set_q(hand_q)

	# log data
	# log_data(sh,hand_q)

	# live steam data
	while True:
		rospy.sleep(0.5)
		log.info(f"{sh.contact_states}")
		pub.publish(sh.tactile_point_cloud)


def log_data(sh: ShadowHand, hand_q: Dict) -> None:

	DESIRED_NUMBER_OF_DATA_POINTS = 100
	REFERENCE_VECTOR = Vector3(0.0, 1.0, 0.0)
	SAMPLING_DT = 0.5  # s.

	json_data = {}

	# create the data directory
	data_dir = create_pkg_dir(__file__, "data/")

	# open the file and clear its content
	json_file = open(data_dir + "stat_" + date.today().strftime("%Y%m%d_%H%M%S") + ".json", "w")
	json_file.truncate(0)

	# wait for the fingers to make contact
	while True:
		rospy.sleep(1)

		# while the fingers are not in contact, wait...
		if not sh.is_in_contact:
			log.warn("waiting for contact of any kind...")
			continue

		# while only some of the fingers are not in contact, wait...
		if not all(f.is_in_contact for f in hand_q.keys()):
			log.warn("One or more fingers have made contact, now we are waiting for the rest... the fingers making contact are")
			continue

		break

	# all fingers have made contact
	# now all fingers have made contact, wait for deformation to happen
	rospy.sleep(2)

	# for each finger
	# log to file (test 10) 1000 normal angle errors + number of contact points
	for i in range(DESIRED_NUMBER_OF_DATA_POINTS):

		log.info(f"Data points sampled: {i}/{DESIRED_NUMBER_OF_DATA_POINTS}...")

		rospy.sleep(SAMPLING_DT)

		json_data[f"t{i}"] = {}

		for f in hand_q.keys():

			# contains the angle errors
			theta_e = []

			# compute the angle errors
			for n in f.contact_state.contact_normals:
				theta_e.append(geometry.angle(n, REFERENCE_VECTOR))

			# fill time step dict
			json_data[f"t{i}"][f.name] = {
				"nc": len(f.contact_state.contact_normals),
				"theta_e": theta_e
			}

	json.dump(json_data, json_file, indent=4)
	log.success("Successfully saved tactile data")


if __name__ == '__main__':
	try:
		rospy.init_node("sr_tactile_perception", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
