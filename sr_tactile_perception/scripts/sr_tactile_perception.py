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

from ros_utils_py.msg import PointCloud
from ros_utils_py.gazebo import gazebo

# init logger
log = Logger()

def main() -> None:
 
	pub = rospy.Publisher("live_plotter", PointCloud,queue_size=1)
 
	# create shadow hand object
	sh : ShadowHand = ShadowHand()
 
	# joint configuration, from base to tip (does this make contact with the pen? yes)
	test_q: list = [0.0, 0.0, math.pi / 2.0]
 
	hand_q = {
		sh.index_finger  : test_q,
		# sh.middle_finger : test_q,
		# sh.ring_finger   : test_q,
		# sh.little_finger : test_q
	}
 
	# set the hand q
	sh.set_q(hand_q)

	# create the data directory
	data_dir = create_pkg_dir(__file__, "data/")

	# open the file and clear its content
	json_file = open(data_dir + date.today().strftime("%Y%m%d_%H%M%S") + ".json", "w")
	json_file.truncate(0)
 
	# object to store contact data in
	json_data = {}

	while True:
		rospy.sleep(1)
  
		# while the fingers are not in contact, wait...
		if not sh.is_in_contact:
			log.warn("waiting for contact of any kind...")
			continue
		
		# while only some of the fingers are not in contact, wait...
		# if not all(f.is_in_contact for f in sh.fingers):
		# 	log.warn("One or more fingers have made contact, now we are waiting for the rest...")
		# 	continue

		# now all fingers have made contact, wait for deformation to happen
		rospy.sleep(2)

		# save the contact information in json_data
		for f in sh.fingers:
			json_data[f.name] = gazebo.point_cloud_to_dict(f.tactile_point_cloud)
	
		# write json_data to file
		json.dump(json_data, json_file, indent=4)

		log.success("Successfully saved tactile data")
		
		# live stream tactile data
		while True:
			kill_on_ctrl_c()
			# log.warn("publishing to live_plotter for live tactile data...")
			for f in sh.fingers:
				pub.publish(f.tactile_point_cloud)


if __name__ == '__main__':
	try:
		rospy.init_node("sr_tactile_perception", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
