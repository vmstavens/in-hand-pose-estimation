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
	sh: ShadowHand = ShadowHand()
 
	# joint configuration, from base to tip (does this make contact with the pen? yes)
	# test_q: list = [0.0, math.pi / 4.0, math.pi / 2.0]
	test_q: list = [math.pi / 8.0, 0.0, math.pi / 2.0]
 
	hand_q = {
		sh.index_finger  : test_q,
		sh.middle_finger : test_q,
		sh.ring_finger   : test_q,
		sh.pinky_finger  : test_q
	}
 
	# set the hand q
	sh.set_q(hand_q)

	# create the data directory
	data_dir = create_pkg_dir(__file__, "data/")

	# open the file and clear its content
	json_file = open(data_dir + "stat_" + date.today().strftime("%Y%m%d_%H%M%S") + ".json", "w")
	json_file.truncate(0)
 
	# object to store contact data in
	json_data = {}

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

	DESIRED_NUMBER_OF_DATA_POINTS = 100

	for f in hand_q.keys():
		json_data[f.name] = {}
		
	while True:

		# save the contact information in json_data
		for f in hand_q.keys():
			tac_pc_dict = gazebo.point_cloud_to_dict(f.tactile_point_cloud)
			json_data[f.name]["normals_x"].extend( tac_pc_dict["normals_x"] )
			json_data[f.name]["normals_y"].extend( tac_pc_dict["normals_y"] )
			json_data[f.name]["normals_z"].extend( tac_pc_dict["normals_z"] )

		if all( [(len(json_data[f.name]["normals_x"]) >= DESIRED_NUMBER_OF_DATA_POINTS) for f in hand_q.keys()] ):
			# write json_data to file
			for f in hand_q.keys():
				json_data[f.name]["normals_x"] = json_data[f.name]["normals_x"][:100]
				json_data[f.name]["normals_y"] = json_data[f.name]["normals_y"][:100]
				json_data[f.name]["normals_z"] = json_data[f.name]["normals_z"][:100]
			json.dump(json_data, json_file, indent=4)
			break

		# live stream tactile data
		# while True:
		# 	kill_on_ctrl_c()
		# 	rospy.sleep(0.1)
		# 	pub.publish(sh.tactile_point_cloud)

	log.success("Successfully saved tactile data")

if __name__ == '__main__':
	try:
		rospy.init_node("sr_tactile_perception", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
