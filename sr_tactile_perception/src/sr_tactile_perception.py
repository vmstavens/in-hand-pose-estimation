#!/usr/bin/env python3
import json
import math
from datetime import datetime
from random import random
from typing import Dict, List, Optional, Union, Tuple

import rospy
from gazebo_msgs.msg import ContactState, ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateResponse, SetModelState
from geometry_msgs.msg import Twist, Vector3, Wrench
from ros_utils_py.geometry import geometry
from ros_utils_py.log import Logger
from ros_utils_py.utils import create_pkg_dir, keep_alive, kill_on_ctrl_c
from shadow_hand import ShadowHand, ShadowFinger, ShadowWrist
from ros_utils_py.pc_utils import PointCloudUtils as pcu
from dynamic_reconfigure.client import Client
import os
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import json
import time
import pandas as pd
# init logger
log = Logger()

# live steam data
# while True:
# 	rospy.sleep(0.5)
# 	log.info(f"{sh.contact_states}")
# 	pub.publish(sh.tactile_point_cloud)

class Data(object):
	"""protected property demo"""

	def __init__(self, sh, data={}, i=0):
			self._data = data
			self._i = i
			self._sh = sh
	#
	@property
	def data(self):
		return self._data

	@property
	def i(self):
		return self._i

	@property
	def sh(self):
		return self._sh
	#

	@data.setter
	def data(self, value):
		self._data = value

	@i.setter
	def i(self, value):
		self._i = value

	@sh.setter
	def sh(self, value):
		self._sh = value

def main() -> None:

	# live plotter
	# pub = rospy.Publisher("live_plotter", PointCloud, queue_size=1)

	# create shadow hand object
	sh: ShadowHand = ShadowHand()

	sh.record_start()

	sh.update_frequency = 0.0
 
	plan = make_probe_plan(sh)
 
	I: int = 0
	for p in plan:
		print(f"{I=}")
		sh.set_q(p,interpolation_time=3,block=True)
		I += 1

	date_time = datetime.fromtimestamp(datetime.timestamp(datetime.now()))
	str_date_time = date_time.strftime("%Y%m%d_%H%M%S")
	sh.record_stop("/home/user/projects/shadow_robot/base/src/in_hand_pose_estimation/sr_tactile_perception/data/post-thesis-tests/" + str_date_time + ".pcd")

	rospy.sleep(4)

	exit()
 
 
def append_data(data: Dict, i: int, sh: ShadowHand) -> Dict:
	# initiate
	data[f"t{i}"] = {}

	for f in sh.fingers:

		# contains the angle errors
		normals, forces, torques, contact_points = {}, {}, {}, {}

		# compute the angle errors
		for j, n in enumerate(f.contact_state.contact_normals):
			normals[f"n{j}"] = [n.x, n.y, n.z]
			forces[f"f{j}"] = [f.contact_state.wrenches[j].force.x, f.contact_state.wrenches[j].force.y, f.contact_state.wrenches[j].force.z]
			torques[f"tau{j}"] = [f.contact_state.wrenches[j].torque.x, f.contact_state.wrenches[j].torque.y, f.contact_state.wrenches[j].torque.z]
			contact_points[f"contact_point{j}"] = [f.contact_state.contact_positions[j].x, f.contact_state.contact_positions[j].y, f.contact_state.contact_positions[j].z]

			# fill time step dict
			data[f"t{i}"][f.name] = {
				"nc": len(f.contact_state.contact_normals),
				"normals": normals,
				"forces": forces,
				"torques": torques,
				"contact_points": contact_points
			}
   
	return data

def make_probe_plan(sh: ShadowHand) -> List[Dict] :
	# joint configuration, from base to tip (does this make contact with the pen? yes)
	open_q: List = [0.0, 0.0, math.pi/2.3]
	close_q: List = [math.pi / 3, math.pi / 3, math.pi / 3]

	sequence: List[Dict] = []

	sequence.append({
		sh.index_finger: [0.0, math.pi / 4, math.pi/4, -math.pi / 4],
		sh.middle_finger: [0.0, math.pi / 4, math.pi/4, -math.pi / 12],
		sh.ring_finger: [0.0, math.pi / 4, math.pi/4, -math.pi / 12],
		sh.little_finger: [0.0, math.pi / 4, math.pi/4, -math.pi / 4],
	})

	wrist_2_min_max = (-0.489, 0.174)
	wrist_1_min_max = (-0.698, 0.489)

	print(f"curve interval {wrist_2_min_max[1]=} | {wrist_2_min_max[0]=} | {0.5 * ( wrist_2_min_max[1] - wrist_2_min_max[0] )=} ------------------------------------------------------------------")

	dphi = math.pi / 32

	# generate sequence
	for theta in np.arange(wrist_2_min_max[1], wrist_2_min_max[0] / 2.0,  - 0.2 * (wrist_2_min_max[1] - wrist_2_min_max[0])):
		sequence.append({
			sh.index_finger:  [0.0, dphi, math.pi/2.2, -math.pi / 4],
			sh.middle_finger: [0.0, dphi, math.pi/2.2, -math.pi / 12],
			sh.ring_finger:   [0.0, dphi, math.pi/2.2, -math.pi / 12],
			sh.little_finger: [0.0, dphi, math.pi/2.2, -math.pi / 4],
			sh.wrist:         [theta, 0.0]
		})
		# sequence.append({sh.wrist : [theta]})
		# sequence.append({sh.wrist : [theta, wrist_1_min_max[1]]})
		sequence.append({sh.wrist: [theta, wrist_1_min_max[0] + 0.4]})
		sequence.append({sh.wrist: [theta, wrist_1_min_max[1] - 0.2]})

		# dphi = dphi + math.pi / 32
		dphi = dphi + math.pi / 16

	sequence.append({
		sh.index_finger: close_q,
		sh.middle_finger: close_q,
		sh.ring_finger: close_q,
		sh.little_finger:close_q,
		sh.wrist: [-math.pi/3],
	})
 
	# sequence.append({})

	# print(" /////// SEQUENCE LENGTHS //////////////")
	# for s in sequence:
	# 	print(f" {[f.name for f in s.keys()]} {[len(l) for l in s.values()]=}")

	# close_dict = {
	# 	sh.index_finger:  close_q,
	# 	sh.middle_finger: close_q,
	# 	sh.ring_finger:   close_q,
	# 	sh.little_finger: close_q,
	# 	sh.wrist        : [1.0]
	# }

	return sequence

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
		log.info(f"Data points sampled: {i + 1}/{DESIRED_NUMBER_OF_DATA_POINTS}.................................................................................................................")

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
	exit()

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
	# getting the timestamp
	date_time = datetime.fromtimestamp(datetime.timestamp(datetime.now()))
	str_date_time = date_time.strftime("%Y%m%d_%H%M%S")
	json_file = open(f"{data_dir}" + f"{str_date_time}" + ".json", "w")
	json_file.truncate(0)
	return json_file

if __name__ == '__main__':
	try:
		rospy.init_node("sr_tactile_perception", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
