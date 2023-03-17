#!/usr/bin/env python3
import json
import math
from datetime import date, datetime
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

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.cfg import TestConfig


# init logger
log = Logger()

def main() -> None:
	
	# server = Server()
 
	# how to setup server
	# http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile
	t = 0
	timeout = 100
	while t < timeout:
		# print(f"{rospy.get_name()} sleeping {t}/{timeout}...")
		rospy.sleep(1)
		t += 1
		test_str = rospy.get_param("~test")
		log.info(f"This is my {rospy.get_name()} parameter: {test_str}...")
	# move_2("cube")
	keep_alive(rospy.get_name())
	
	# return


# rostopic pub - 1 /gazebo/set_model_state gazebo_msgs/ModelState "model_name: 'cube'
# pose:
#   position:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   orientation:
#     x: 0.0
#     y: 0.0
#     z: 0.0


def move_2(prop_name: str) -> None:

	# set_model_state_topic = "/gazebo/set_model_state"
	model_twist_topic     = "/cube/cmd_vel"

	# Create a publisher to publish the twist message
	twist_pub = rospy.Publisher(model_twist_topic, Twist, queue_size=1)
 
	# Create a client to call the SetModelState service
	# set_state_client = rospy.ServiceProxy(set_model_state_topic, SetModelState)

	# Create a ModelState message to set the state of the model
	# model_state = ModelState()
	# model_state.model_name = prop_name
	# model_state.reference_frame = 'world'
	# model_state.reference_frame = 'base_link'
	# model_state.twist.linear.y = 0.5

	# Publish the twist message to move the robot
	twist_msg = Twist()
	twist_msg.linear.y = 0.1
	twist_pub.publish(twist_msg)
	print(f"I have published to {model_twist_topic}")

	# Call the SetModelState service to set the model state
	# resp = set_state_client(model_state)
	# print(resp)
	return


if __name__ == '__main__':
	try:
		rospy.init_node("test", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
