#!/usr/bin/env python3
import json
import math
from datetime import datetime
from random import random
from typing import Dict, List, Optional

import rospy
from gazebo_msgs.msg import ContactState, ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateResponse, SetModelState
from geometry_msgs.msg import Twist, Vector3, Wrench
from ros_utils_py.geometry import geometry
from ros_utils_py.log import Logger
from ros_utils_py.msg import PointCloud
from ros_utils_py.utils import create_pkg_dir, keep_alive, kill_on_ctrl_c
from shadow_hand import ShadowHand

from dynamic_reconfigure.client import Client


# init logger
log = Logger()

# live steam data
# while True:
# 	rospy.sleep(0.5)
# 	log.info(f"{sh.contact_states}")
# 	pub.publish(sh.tactile_point_cloud)


def main() -> None:

	# live plotter
	# pub = rospy.Publisher("live_plotter", PointCloud, queue_size=1)

	# create shadow hand object
	sh: ShadowHand = ShadowHand()

	# joint configuration, from base to tip (does this make contact with the pen? yes)
	test_q: list = [math.pi/16.0, 0.0, math.pi / 2.0]
	# test_q: list = [math.pi / 8.0, 0.0, math.pi / 2.0]

	hand_q = {
		sh.index_finger: test_q,
		sh.middle_finger: test_q,
		sh.ring_finger: test_q,
		sh.little_finger: test_q,
	}

	experiment_config = {
		"num_of_dp"      : 100,                                  # number of data points
		"v_ref"          : [0.0, 1.0, 0.0],                      # the reference vector for error calc
		"dt"             : 0.5,                                  # the time between data collection
		"prop_name"      : rospy.get_param("/prop_name"),             # prop we are testing on
		"prop_mechanics" : rospy.get_param("/prop_mechanics") # contact mechanics (static or dynamic)
	}

	# set the hand q
	sh.set_q(hand_q)

	# # log data
	if experiment_config["prop_name"] != "sphere" or experiment_config["prop_name"] != "stanford_bunny":
		wait_for_stable_contact(sh, hand_q)

	# while True:
	# 	rospy.sleep(1)
	# 	log.info(sh.contact_states[sh.index_finger])

	log_data(sh,hand_q,experiment_config)
 
	keep_alive(rospy.get_name())
 

def log_data(sh: ShadowHand, fingers: Dict, experiment_config: Dict) -> None:


	DESIRED_NUMBER_OF_DATA_POINTS = experiment_config["num_of_dp"]
	SAMPLING_DT                   = experiment_config["dt"]
	PROP_NAME                     = experiment_config["prop_name"]
	json_data = {}
	json_data["experiment_config"] = experiment_config

	# create the data directory
	data_dir = create_pkg_dir(__file__, "data/")

	# open the file and clear its content
	log_file = create_log_file(data_dir)


	# now all fingers have made contact, wait for deformation to happen
	rospy.sleep(2)

	for i in range(DESIRED_NUMBER_OF_DATA_POINTS):

		# progress bar....
		log.info(f"Data points sampled: {i + 1}/{DESIRED_NUMBER_OF_DATA_POINTS}...")

		# sampling time
		rospy.sleep(SAMPLING_DT)
  
		# initiate
		json_data[f"t{i}"] = {}

		for f in fingers.keys():

			# contains the angle errors
			normals, forces, torques, contact_points = {}, {}, {}, {}

			# compute the angle errors
			for j, n in enumerate(f.contact_state.contact_normals):
				normals[f"n{j}"] = [n.x, n.y, n.z]
				forces[f"f{j}"] = [f.contact_state.wrenches[j].force.x, f.contact_state.wrenches[j].force.y, f.contact_state.wrenches[j].force.z]
				torques[f"tau{j}"] = [f.contact_state.wrenches[j].torque.x, f.contact_state.wrenches[j].torque.y, f.contact_state.wrenches[j].torque.z]
				contact_points[f"contact_point{j}"] = [f.contact_state.contact_positions[j].x, f.contact_state.contact_positions[j].y, f.contact_state.contact_positions[j].z]

			# fill time step dict
			json_data[f"t{i}"][f.name] = {
				"nc": len(f.contact_state.contact_normals),
				"normals"        : normals,
				"forces"         : forces,
				"torques"        : torques,
				"contact_points" : contact_points
			}

	json.dump(json_data, log_file, indent=4)
	log.success("Successfully saved tactile data")


def wait_for_stable_contact(sh: ShadowHand, q: Dict) -> bool:
	# wait for the fingers to make contact
	timeout = 100
	t = 0
	while t < timeout:
		t += 1
		rospy.sleep(0.5)
		# while the fingers are not in contact, wait...
		if not sh.is_in_contact:
			log.warn("waiting for contact of any kind...")
			continue

		# while only some of the fingers are not in contact, wait...
		if not all(f.is_in_contact for f in q.keys()):
			log.warn("One or more fingers have made contact, now we are waiting for the rest... the fingers making contact are")
			continue

		return True

	return False

def create_log_file(data_dir: str):
	date = datetime.now()
	json_file = open(data_dir + "stat_" + date.today().strftime(f"{date.year}{date.month}{date.day}_{date.hour}{date.minute}{date.second}") + ".json", "w")
	json_file.truncate(0)
	return json_file

if __name__ == '__main__':
	try:
		rospy.init_node("sr_tactile_perception", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
