#!/usr/bin/env python3

from typing import Dict, List, Optional, Tuple, Union
from sr_robot_commander.sr_hand_commander import SrHandCommander
import rospy
from gazebo_msgs.msg import ContactsState, ContactState
from geometry_msgs.msg import Vector3, Wrench
import time

from ros_utils_py.log import Logger
from ros_utils_py.pc_utils import PointCloudUtils as pcu
from ros_utils_py.geometry import geometry as geo
from ros_utils_py.utils import COLORS_RGBA, COLOR
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np
from math import sqrt
from sklearn.cluster import DBSCAN

class ShadowFinger:
	"""A wrapper class for interacting with the individual fingers of a Shadow Dexterous Hand"""

	class FINGERS_NAMES():
		"""Enum for shorthand finger strings"""
		THUMB_FINGER = "th"
		INDEX_FINGER = "ff"
		MIDDLE_FINGER = "mf"
		RING_FINGER = "rf"
		LITTLE_FINGER = "lf"

	def __init__(self, finger_type: str, hc: SrHandCommander, update_freq: float = 2.0, chirality: str = "rh"):
		
		# logger
		self.__log = Logger()
  
		self.__hand_commander: SrHandCommander           = hc
		self.__update_freq: float                        = update_freq
		self.__finger_type: str                          = finger_type
		self.__chirality: str                            = chirality
		self.__tac_topic: str                            = f"/contacts/{chirality}_{finger_type}/distal"
		self.__contact_states_sample: List[ContactState] = []
		self.__contact_state: Optional[ContactState]     = ContactState()
		self.__CONTACT_STATES_SAMPLE_THRESHOLD           = 11 # with the plugin publishing at ~100 Hz, 11 is about once every 10 ms or 10 times pr. second
		self.__SAME_POINT_DISTANCE_THRESHOLD             = 0.0002  # m i.e. 0.2 mm
		self.__contact_state_sanitized                   = ContactState()
		self.__name: str                                 = f"{self.__chirality}_{self.__finger_type.upper()}"
		self.__tactile_point_cloud: pcu.PointCloud       = pcu.PointCloud()

		# look up table for colors depending on finger type
		self.__fingers_and_colors: Dict[str, COLOR] = {
			"TH": COLORS_RGBA.MAGENTA, 
			"FF": COLORS_RGBA.BLUE   , 
			"MF": COLORS_RGBA.GREEN  , 
			"RF": COLORS_RGBA.YELLOW , 
			"LF": COLORS_RGBA.RED    }
  
		# set the finger color code
		self.__finger_color: COLOR = self.__fingers_and_colors[self.__finger_type.upper()]
  
		# the label of each finger and how many joints they each have
		self.__fingers_and_num_of_joints = {"TH": 5, "FF": 4, "MF": 4, "RF": 4, "LF": 5}
  
		# set the number of joints in the finger
		self.__num_of_joints = self.__fingers_and_num_of_joints[self.__finger_type.upper()]

		# there are 4 joint, in range the last number is exclusive. for example ["rh_FFJ1", "rh_FFJ2", "rh_FFJ3", ... ]
		self.__joint_names = [ f"{self.__chirality}_{self.__finger_type.upper()}J{i}" for i in range(1,self.__num_of_joints + 1) ] 
  
		# subscribe and print the found contacts
		if self.__is_biotac_sim_live:
			# runs slow and sets __contact_state to a sanitized contact state
			self.__tac_sub        = rospy.Subscriber(f"/contacts/{self.__chirality}_{self.__finger_type}/distal", ContactsState, callback = self.__read_biotac_sim, queue_size = 1)
			# runs fast and creates a sanitized contact state
			self.__tac_sub_sanitize = rospy.Subscriber(f"/contacts/{self.__chirality}_{self.__finger_type}/distal", ContactsState, callback=self.__sanitize_biotac_sim, queue_size=1)
			self.__is_contact_states_sample_full: bool = False
		
		# https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/documentation_health_metrics/user_guide/sd_robot_commander.html
		# The limits in the current implementation of the firmware are from 200 to 1000 (measured in custom units)
		self.__max_force: List[float] = [600.0 for j in self.__joint_names]  # cu (custom units)
  
  
	@property
	def name(self) -> str:
		"""finger's name"""
		return self.__name

	@property
	def finger_color(self) -> COLOR:
		"""the color associated with this finger in RGB format in a tuple"""
		return self.__finger_color
  
	@property
	def joint_names(self) -> List[str]:
		"""joint names"""
		return self.__joint_names

	@property
	def number_of_joints(self) -> int:
		"""gets the number of joints for this particular finger"""
		return self.__num_of_joints

	@property
	def update_freq(self) -> float:
		"""get the tactile data update frequency in Hz"""
		return self.__update_freq

	@update_freq.setter
	def update_freq(self, new_update_freq: float) -> None:
		"""get the tactile data update frequency in Hz"""
		self.__update_freq = new_update_freq

	@property
	def tac_type(self) -> str:
		"""tactile sensor type. Two options are present: biotac_sim and None"""
		return "biotac_sim" if self.__is_biotac_sim_live() else "None"

	@property
	def tac_topic(self) -> str:
		"""topic this finger reads from to get its tactile data"""
		return self.__tac_topic

	@property
	def q(self) -> List[float]:
		"""get the current joint configuration"""
		return [self.__hand_commander.get_joints_position()[jn] for jn in self.joint_names]

	@property
	def contact_state(self) -> Optional[ContactState]:
		"""current tactile data, it returns empty i.e. [] if no contact is made"""
		return self.__contact_state

	@contact_state.setter
	def contact_state(self, new_cs: Optional[ContactState]) -> Optional[ContactState]:
		"""set tactile data, it returns empty i.e. [] if no contact is made"""
		self.__contact_state = new_cs

	@property
	def is_in_contact(self) -> bool:
		"""returns if the finger is in contact"""
		return False if (len(self.contact_state.contact_positions) == 0) else True

	@property
	def tactile_point_cloud(self) -> pcu.PointCloud:
		"""returns the tactile point cloud from the finger in contact"""
		return self.__tactile_point_cloud

	@property
	def max_force(self) -> List[float]:
		"""the maximum force excretable by this finger as an array of floats. The limits in the current implementation of the firmware are from 200 to 1000 (measured in custom units)"""
		return self.__max_force

	@max_force.setter
	def max_force(self, new_max_force: List[Optional[float]]) -> None:
		"""sets the maximum force excretable by this finger as an array of floats"""
		new_max_force_valid: List[float] = self.make_valid_max_force(new_max_force)
		self.__log.info(f"{new_max_force_valid=}")

		is_force_valid = all([nmf >= 200 and nmf <= 1000 for nmf in new_max_force_valid])
		if not is_force_valid:
			raise ValueError("the maximum force of a joint cannot exceed 1000 or go below 200 custom units")
		for i, j in enumerate(self.__joint_names):
			self.__log.info(f"{j=} , {new_max_force_valid[i]=}")
			self.__hand_commander.set_max_force(j,new_max_force_valid[i])
		self.__max_force = new_max_force_valid

	@property
	def contact_states_sample_threshold(self):
		return self.__CONTACT_STATES_SAMPLE_THRESHOLD

	@contact_states_sample_threshold.setter
	def contact_states_sample_threshold(self, new_threshold: int) -> None:
		self.__CONTACT_STATES_SAMPLE_THRESHOLD = new_threshold

	def make_valid_max_force(self,mf:List[Optional[float]]) -> List[float]:
		valid_mf = []

		# in this case the input is just not valid, and throw an error
		if len(mf) > self.number_of_joints:
			raise ValueError(f"You cannot set more joints than what the finger has. Finger: {self.__chirality}, num of joints: {self.number_of_joints}, length of given q: {len(q)}")

		# if a q is given with a shorter length than number_of_joints, pad q with the joint's current values
		elif len(mf) < self.number_of_joints:

			# overwrite None values in valid_q with self.q values
			for i,mfi in enumerate(mf):
				valid_mf.append(mfi if mfi is not None else self.max_force[i])

			# expand valid_q with self.q values
			d_length = self.number_of_joints - len(mf)

			for i in range(d_length):
				valid_mf.append(self.max_force[len(mf) + i - 1])
			return valid_mf
		else:
			return mf

	def make_valid_q(self,q: List[Optional[float]]) -> List[float]:
		"""this function converts a q into a des_q which has the correct number of elements and constants joint values for Nones in q e.g.
		make_valid_q( [0.0, None, 0.0, pi] ) -> des_q = [0.0, current_q, 0.0, pi, current_q]
		"""

		valid_q = [qi for qi in q]
  
		# in this case the input is just not valid, and throw an error
		if len(q) > self.number_of_joints:
			raise ValueError(f"You cannot set more joints than what the finger has. Finger: {self.__name}, num of joints: {self.number_of_joints}, length of given q: {len(q)}")

		# if a q is given with a shorter length than number_of_joints, pad q with the joint's current values
		if len(q) < self.number_of_joints:
			# expand valid_q with self.q values
			d_length = self.number_of_joints - len(q)
			for i in range(d_length):
				valid_q.append(self.q[len(valid_q)])

		# overwrite None values in valid_q with self.q values
		for i, vq in enumerate(valid_q):
			valid_q[i] = vq if vq is not None else self.q[i]

		return valid_q

	def set_q(self, q: List[Optional[float]], interpolate_time:int = 1, block:bool = False) -> bool:
		"""the finger's q is set in the order: tip_q, q2, ... and base_q. If a None is given for a particular angle, the angle remains constant.
		Returns:
			bool: did the set function succeed
		"""

		joint_trajectory = self.__make_jt(q,interpolation_time=interpolate_time)

		try:
			self.__log.warn(rospy.get_name() + f" attempting to set {self.name} joint values {self.make_valid_q(q)}")
			# self.__hand_commander.run_joint_trajectory(joint_trajectory=joint_trajectory)
			self.__hand_commander.run_joint_trajectory_unsafe(joint_trajectory=joint_trajectory,wait=block)
			self.__log.success(rospy.get_name() + " succeeded...")
			return True
		except:
			self.__log.error(f"failed to set {self.name} joint values {q}...")
			return False

	def wait_for_contact(self, timeout: float = 5.0) -> Optional[ContactState]:
		"""gets the current tactile data, or wait until something is available. The waiting time is determined by the timeout argument

		Returns:
			ContactState | None: list of contact points or None
		"""
		start = rospy.get_rostime()

		# while None and an empty array is received, keep looking
		while self.contact_state is None:

			now = rospy.get_rostime()

			if (now.to_sec() - start.to_sec()) > timeout:
				self.__log.warn(f"Tactile data was not received within the timeout of {timeout}... retuning with an empty array")
				return None

			self.__log.warn("awaiting contact points...")
			if (self.__update_freq == 0): 
				continue 
			else:
				time.sleep(1.0 / self.__update_freq)

			continue

		return self.contact_state

	def save_tactile_point_cloud(self, path_to_file: str) -> bool:
		return True

	def __is_biotac_sim_live(self) -> bool:
		"""check if the biotac_sim_plugin package has been loaded"""
		published_topics = rospy.get_published_topics()
		does_the_topic_exist_list = [s[0] == self.__tac_topic for s in published_topics]
		result = any(does_the_topic_exist_list)
		if result:
			self.__log.success("the biotac_sim_plugin topics have successfully been found....")
			return True
		else:
			self.__log.error("the biotac_sim_plugin topics have NOT been found....")
			return False

	def __get_finger_type_in_collision_strings(self, collision1_name: str, collision2_name: str) -> str:
		""" contact states are given independent of which finger experiences it. We therefore filter 
			the collision strings for the finger identifier e.g. ff, mf etc.
			since two strings are given, one for the prop and one for the finger, we need to identify 
			the string with the finger in e.g.

			collision name 1: usrh::rh_rfdistal::rh_rfdistal_collision  : finger collision string example
			collision name 2: pen_black::link_13::collision             : prop collision string example

		Args:
			collision1_name (str): collision string 1
			collision2_name (str): collision string 2

		Returns:
			str: the finger type found in the collision string
		"""
		l_collision1_name = collision1_name.split("::")
		l_collision2_name = collision1_name.split("::")
		finger_collision_name = collision1_name if (("distal" in collision1_name) and ("us" in collision1_name)) else collision2_name
		l_finger_collision_name = finger_collision_name.split("::")
		finger_type = l_finger_collision_name[1].replace("distal", "").split("_")[1]
		return finger_type

	def __read_biotac_sim(self, data: Optional[ContactsState]) -> None:
		"""the callback function used to sample tactile data from biotac_sim_plugin. It takes a Gazebo ContactsState, which is a list of contact states, each finger has a contact state with multiple points with data"""

		# wait for the sample list to fill up
		if not self.__is_contact_states_sample_full:
			return

		# reconcile contact states such that all normals and other vectors are pointing in the correct direction
		self.__contact_state = self.__contact_state_sanitized

		# build the tactile point cloud from displaced points of contact with the depth of the deformation along the normal vector

		# contact points
		P: Optional[List[Vector3]] = self.__contact_state.contact_positions

		# contact normals
		N: Optional[List[Vector3]] = self.__contact_state.contact_normals

		# depths
		D: List[float] = self.__contact_state.depths

		# displaced tactile point cloud, the values above (P, N and D) are just used to make the line below more readable...
		self.__tactile_point_cloud.points = np.array([(p.x + (-N[i].x * D[i]), p.y + (-N[i].y * D[i]), p.z + (-N[i].z * D[i])) for i, p in enumerate(P)])
		# self.__tactile_point_cloud.positions = [Vector3(p.x + (-N[i].x * D[i]), p.y + (-N[i].y * D[i]), p.z + (-N[i].z * D[i])) for i, p in enumerate(P)]

		# fill color array
		self.__tactile_point_cloud.colors = [ self.finger_color.color_code for i in P ]
		# self.__tactile_point_cloud.colors = [ Color(self.finger_color.label, self.finger_color.color_code) for i in P ]

		# pass along the remaining values from the contact state the data, to create the tactile point cloud. Since the normals are given from the finger, the normals are flipped here to represent the surface normals
		self.__tactile_point_cloud.normals = [ ( -1.0*n.x, -1.0*n.y, -1.0*n.z) for n in self.__contact_state.contact_normals]
		# self.__tactile_point_cloud.normals = [ Vector3( -1.0*n.x, -1.0*n.y, -1.0*n.z) for n in self.__contact_state.contact_normals]
		# self.__tactile_point_cloud.normals = self.contact_state.contact_normals

		# save if the tactile point cloud is empty
		# self.__tactile_point_cloud.empty   = (len(self.contact_state.contact_positions) == 0)
  
		# reset contact_states_sample
		self.__contact_states_sample.clear()
		self.__is_contact_states_sample_full = False

	def __sanitize_biotac_sim(self, data: Optional[ContactsState]) -> None:
		"""This is a callback function which runs at ~100Hz. It builds a lit of contacts states until the list contains enough elements, sanitizes them and returns a correct contact state"""

		# return if no data is received yet
		if len(data.states) == 0: return

		# if the update frequency is set to 0
		# if (self.__update_freq == 0):
		# 	self.__contact_state_sanitized = self.__contact_state
		# 	self.__

		# when we have filled up the array with "self.__contact_state_sample_size" elements
		if len(self.__contact_states_sample) >= self.__CONTACT_STATES_SAMPLE_THRESHOLD:
			self.__contact_state_sanitized = self.__sanitize_contact_states(self.__contact_states_sample)
			self.__is_contact_states_sample_full = True
			return
		else:
			# if contact has been made, append the correct contact state to __contact_states_sample
			for cs in data.states:
				# among the received contact states, append the one related to this finger
				if (self.__get_finger_type_in_collision_strings(cs.collision1_name, cs.collision2_name) == self.__finger_type):
					self.__contact_states_sample.append(cs)
					break
		return

	def __sanitize_contact_states(self, css: List[ContactState]) -> Optional[ContactState]:

		def __index_of_sameish_point(cp: Vector3, cs: ContactState,csi: int, dist_threshold: float) -> Optional[Tuple[int,int]]:
			"""returns the index of the contact point closest to cp in cs, None is returned if no distance is below dist_threshold"""
			closest_point_index = 0
			shortest_dist = 1000 # 1km distance, should be more than enough
			for i, cpi in enumerate(cs.contact_positions):
				# first we find the point closest to the one provided
				if geo.l2_dist(cp, cpi) < shortest_dist:
					closest_point_index = i
					shortest_dist = geo.l2_dist(cp, cpi)
			# we then determine if the found distance is sufficiently low to be considered the same point
			if shortest_dist <= dist_threshold:
				return (csi,closest_point_index)
			return None

		def __euclidean_clustering(points, eps, min_samples):
			"""
			Perform Euclidean clustering on a list of 3D points.
			
			Parameters:
			points (list of tuples): A list of 3D points.
			eps (float): The maximum distance between two points for them to be considered as in the same neighborhood.
			min_samples (int): The minimum number of points in a neighborhood for it to be considered as a cluster.
			
			Returns:
			list: The indices of the points in the largest cluster.
			"""
			# Create a numpy array from the list of points
			X = np.array(points)
			
			# Perform DBSCAN clustering
			dbscan = DBSCAN(eps=eps, min_samples=min_samples)
			dbscan.fit(X)
			
			# Extract the cluster labels
			labels = dbscan.labels_
   
			# in case the number of contact points is so small, no clusters can be defined
			if all([l == -1 for l in labels]):
				return []
			
			# Extract the indices of the points in the largest cluster
			cluster_sizes = np.bincount(labels[labels != -1])
			largest_cluster_label = np.argmax(cluster_sizes)
			largest_cluster_indices = np.where(labels == largest_cluster_label)[0]
			# Return the indices of the points in the largest cluster
			largest_cluster_points = [points[i] for i in largest_cluster_indices.tolist()]

			return largest_cluster_points

		def __vec2tuple(v: Vector3) -> Tuple[float,float,float]:
			if type(v) is tuple:
				return v
			else:
				return (v.x,v.y,v.z)

		def __tuple2vec(t: Tuple) -> Vector3:
			if type(t) is tuple:
				return Vector3(t[0], t[1], t[2])
			else:
				raise ValueError(f"Wrong input type in __tuple2vec, expected tuple, got {type(t)}")

		def __calculate_centroid(points: List[Tuple]):
			"""
			Calculate the centroid of a set of 3D points.
			
			Parameters:
			points (list of tuples): A list of 3D points.
			
			Returns:
			tuple: The x, y, and z coordinates of the centroid.
			"""
			# Convert the list of points to a numpy array
			points_array = np.array(points)
			
			# Calculate the mean of the x, y, and z coordinates
			x_mean = np.mean(points_array[:,0])
			y_mean = np.mean(points_array[:,1])
			z_mean = np.mean(points_array[:,2])
			
			# Return the centroid as a tuple
			return (x_mean, y_mean, z_mean)

		def __is_empty(l: List) -> bool:
			return len(l) == 0

		if __is_empty(css):
			print(f"css is empty")
			return ContactState()

		best_cs = ContactState()
		# find the best contact state based on the number of contact points and its index in the list of contact states
		for i, cs in enumerate(css):
			if len(cs.contact_positions) > len(best_cs.contact_positions):
				best_cs = cs

		if len(best_cs.contact_positions) < 5:
			return best_cs

		for i, cp in enumerate(best_cs.contact_positions):

			# contains tuples of contact state indices and indices for data i.e. normals, points, forces and torques e.g. (1,2) meaning contact state 1/self.__CONTACT_STATES_SAMPLE_THRESHOLD, contact normal at index 2
			index_tuples: List[Tuple[int, int]] = []

			# fill the index tuples list
			for csi, cs in enumerate(css):
				corr_tup: Optional[Tuple[int, int]] = __index_of_sameish_point(cp, cs, csi, self.__SAME_POINT_DISTANCE_THRESHOLD)
				if corr_tup is None:
					continue
				else:
					index_tuples.append(corr_tup)

			# loop through the same point in all contact states paired with their contact state index
			same_normals, same_forces, same_torques = [], [], []
			for csi, ni in index_tuples:
				same_normal: Vector3 = css[csi].contact_normals[ni]
				same_force:  Vector3 = css[csi].wrenches[ni].force
				same_torque: Vector3 = css[csi].wrenches[ni].torque
    
				same_normals.append(same_normal)
				same_forces.append(same_force)
				same_torques.append(same_torque)

			# convert all lists to tuples
			same_normals = [__vec2tuple(n) for n in same_normals ]
			same_forces  = [__vec2tuple(f) for f in same_forces  ]
			same_torques = [__vec2tuple(t) for t in same_torques ]
   
			# compute the largest cluster of normals
			largest_normals_cluster_points = __euclidean_clustering(same_normals, 0.01, 3)
			largest_forces_cluster_points = __euclidean_clustering(same_forces,   0.01, 3)
			largest_torques_cluster_points = __euclidean_clustering(same_torques, 0.01, 3)

			if __is_empty(largest_normals_cluster_points) or __is_empty(largest_forces_cluster_points) or __is_empty(largest_torques_cluster_points):
				return best_cs

			new_normal = __calculate_centroid(largest_normals_cluster_points)
			new_force = __calculate_centroid(largest_normals_cluster_points)
			new_torque = __calculate_centroid(largest_normals_cluster_points)

			best_cs.contact_normals[i] = __tuple2vec(new_normal)
			best_cs.wrenches[i].force = __tuple2vec(new_force)
			best_cs.wrenches[i].torque = __tuple2vec(new_torque)

		return best_cs

	def __make_jt(self, q: List[Optional[float]], interpolation_time: int = 1) -> JointTrajectory:
     
		# 1) get all joint names in a dict
		hand_state = self.__hand_commander.get_current_state()

		valid_q = self.make_valid_q(q)
		for i, jn in enumerate(self.joint_names):
			hand_state[jn] = valid_q[i]
  
		all_joint_names = [jn for jn, jv in hand_state.items()]
		all_joint_values = [jv for jn, jv in hand_state.items()]
		
		jt = JointTrajectory()
		jt.header.frame_id = ''
		jt.joint_names = all_joint_names
  
		point = JointTrajectoryPoint()
		point.positions = all_joint_values
		point.velocities = []
		point.accelerations = []
		point.effort = []
		point.time_from_start = rospy.Duration(secs=interpolation_time)
		jt.points.append(point)
		return jt