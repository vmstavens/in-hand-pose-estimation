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
from ros_utils_py.utils import create_pkg_dir, keep_alive, kill_on_ctrl_c
from shadow_hand import ShadowHand

# init logger
log = Logger()

# rostopic pub -1 / gazebo/set_model_state gazebo_msgs/ModelState "model_name: 'cube'
# pose:
#   position:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   orientation:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#     w: 0.0
# twist:
#   linear:
#     x: 0.0
#     y: 1.0
#     z: 0.0
#   angular:
#     x: 0.0
#     y: 0.0
#     z: 0.0
# reference_frame: ''"
# publishing and latching message for 3.0 seconds


def get_model_state(model_name):
	# rospy.init_node('get_model_state')
	model_state_topic = '/gazebo/get_model_state'
	model_state_msg = ModelState()
	model_state_msg.model_name = model_name

	# Create a rospy Subscriber to get the model state
	model_state_sub = rospy.Subscriber(model_state_topic, ModelState, callback)

	# Wait for the first message to arrive
	t = 0
	while not rospy.is_shutdown() and not model_state_msg.pose and t < 10:
		t += 1
		rospy.sleep(0.1)

	# Unsubscribe from the topic
	model_state_sub.unregister()

	return model_state_msg


def callback(model_state):
	global model_state_msg
	model_state_msg = model_state


def set_model_state(model_state):
	model_state_topic = '/gazebo/set_model_state'
	model_state_pub = rospy.Publisher(model_state_topic, ModelState, queue_size=1)
	print("--- insert model state and make sure the states are not relative ---")
	# Publish the model state message
	model_state_pub.publish(model_state)


def attempt1():
	model_state = get_model_state("cube")
	# Example usage
	# model_state.model_name = 'cube'
	# model_state.pose.position.x = 1.0
	# model_state.twist.linear.x = 1.0
	set_model_state(model_state)
	# why state is fucked?

def main() -> None:
	sleep_time = 10
	log.info(f"going to sleep for {sleep_time}")
	rospy.sleep(sleep_time)
	log.info("Just woke up...")
	attempt1()
	keep_alive(rospy.get_name())

if __name__ == '__main__':
	try:
		rospy.init_node("test", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
