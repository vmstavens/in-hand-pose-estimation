#!/usr/bin/env python3

from typing import Dict, List, Optional, Tuple
from sr_robot_commander.sr_hand_commander import SrHandCommander
import rospy
from gazebo_msgs.msg import ContactsState, ContactState
from geometry_msgs.msg import Vector3
import time

from ros_utils_py.log import Logger
from ros_utils_py.msg import PointCloud
from ros_utils_py.gazebo import gazebo
from ros_utils_py.utils import COLORS_RGBA

from std_msgs.msg import ColorRGBA

class ShadowFinger:
	"""A wrapper class for interacting with the individual fingers of a Shadow Dexterous Hand"""

	class FINGERS_NAMES():
		"""Enum for shorthand finger strings"""
		THUMB_FINGER  = "th"
		INDEX_FINGER  = "ff"
		MIDDLE_FINGER = "mf"
		RING_FINGER   = "rf"
		PINKY_FINGER  = "lf"

	def __init__(self, finger_type: str, hc: SrHandCommander, update_freq: float = 2.0, chirality: str = "rh"):
		
		# logger
		self.__log = Logger()
  
		self.__hand_commander: SrHandCommander = hc
		self.__update_freq: float = update_freq
		self.__finger_type: str = finger_type
		self.__chirality: str = chirality
		self.__tac_topic: str = f"/contacts/{chirality}_{finger_type}/distal"
		self.__contact_state: Optional[ContactState] = ContactState()
		self.__name: str = f"{self.__chirality}_{self.__finger_type.upper()}"
		self.__tactile_point_cloud: PointCloud = PointCloud()

		# look up table for colors depending on finger type
		self.__fingers_and_color_codes: Dict[str, Tuple] = {
			"TH": COLORS_RGBA.MAGENTA, 
			"FF": COLORS_RGBA.BLUE   , 
			"MF": COLORS_RGBA.GREEN  , 
			"RF": COLORS_RGBA.YELLOW , 
			"LF": COLORS_RGBA.RED    }
  
		# set the finger color code
		self.__color_code: Tuple = self.__fingers_and_color_codes[self.__finger_type.upper()]
  
		# the label of each finger and how many joints they each have
		self.__fingers_and_num_of_joints = {"TH": 5, "FF": 4, "MF": 4, "RF": 4, "LF": 5}
  
		# set the number of joints in the finger
		self.__num_of_joints = self.__fingers_and_num_of_joints[self.__finger_type.upper()]

		# there are 4 joint, in range the last number is exclusive
		self.__joint_names = [ f"{self.__chirality}_{self.__finger_type.upper()}J{i}" for i in range(1,self.__num_of_joints + 1) ] 
  
		# subscribe and print the found contacts
		if self.__is_biotac_sim_live:
			self.__tac_sub = rospy.Subscriber(f"/contacts/{self.__chirality}_{self.__finger_type}/distal", ContactsState, callback = self.__read_biotac_sim, queue_size = 1)

	@property
	def name(self) -> str:
		"""finger's name"""
		return self.__name

	@property
	def color_code(self) -> Tuple[float,float,float]:
		"""the color associated with this finger in HLS format in a tuple"""
		return self.__color_code
  
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
		return False if len(self.contact_state.contact_positions) == 0 else True

	@property
	def tactile_point_cloud(self) -> PointCloud:
		"""returns the tactile point cloud from the finger in contact"""
		return self.__tactile_point_cloud

	def make_valid_q(self,q: List[Optional[float]]) -> List[float]:
		"""this function converts a q into a des_q which has the correct number of elements and constants joint values for Nones in q e.g.
		make_valid_q( [0.0, None, 0.0, pi] ) -> des_q = [0.0, current_q, 0.0, pi, current_q]
		"""

		valid_q = []
  
		# in this case the input is just not valid, and throw an error
		if len(q) > self.number_of_joints:
			raise ValueError(f"You cannot set more joints than what the finger has. Finger: {self.__chirality}, num of joints: {self.number_of_joints}, length of given q: {len(q)}")

		# if a q is given with a shorter length than number_of_joints, pad q with the joint's current values
		elif len(q) < self.number_of_joints:
			
			# overwrite None values in valid_q with self.q values
			for i, qi in enumerate(q):
				valid_q.append(qi if qi is not None else self.q[i])

			# expand valid_q with self.q values
			d_length = self.number_of_joints - len(q)

			for i in range(d_length):
				valid_q.append(self.q[len(q) + i - 1])
			return valid_q
		else:
			return q

	def set_q(self, q: List[Optional[float]]) -> bool:
		"""the finger's q is set in the order: tip_q, q2, ... and base_q. If a None is given for a particular angle, the angle remains constant.
		Returns:
			bool: did the set function succeed
		"""

		valid_q = self.make_valid_q(q)
  
		# create the joint dictionary
		q_dict = dict(zip(self.__joint_names, valid_q))
  
		# check if the dictionary has any None, if they do, replace it with the joint's current value
		for key in q_dict.keys():
			if q_dict[key] is None:
				q_dict[key] = self.q[self.joint_names.index(key)]
  
		try:
			self.__log.warn(rospy.get_name() + f" attempting to set {self.name} joint values {valid_q}...")
			self.__hand_commander.move_to_joint_value_target(q_dict)
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
			time.sleep(1.0 / self.__update_freq)
			continue

		return self.contact_state

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

	def __read_biotac_sim(self, data: Optional[ContactsState]) -> None:
		"""the callback function used to sample tactile data from biotac_sim_plugin

		Args:
			data (ContactsState): a Gazebo ContactsState, which is a list of contact states, each finger has a contact state with multiple points with data
		"""

		# sleep
		time.sleep( 1.0 / self.__update_freq )
  
		# if no data is received yet
		if len(data.states) == 0:
			self.__tactile_point_cloud = PointCloud(empty=True)
			self.__contact_state = ContactState()
			return

		# if contact has been made, set this finger's contact state to the correct received one
		for cs in data.states:
			# if this finger is one of the ones in collision
			if (self.__get_finger_type_in_collision_strings(cs.collision1_name, cs.collision2_name) == self.__finger_type):
				self.contact_state = cs
				break

		# reconcile contact states such that all normals and other vectors are pointing in the correct direction
		self.contact_state = self.__reconcile_contact_state(self.contact_state)

		# build the tactile point cloud from displaced points of contact with the depth of the deformation along the normal vector
  
		# contact points
		P: List[Vector3] = self.contact_state.contact_positions
  
		# contact normals
		N: List[Vector3] = self.contact_state.contact_normals
  
		# depths
		D: List[float]   = self.contact_state.depths
  
		# displaced tactile point cloud, the values above (P, N and D) are just used to make the line below more readable...
		self.__tactile_point_cloud.positions = [Vector3(p.x + (-N[i].x * D[i]), p.y + (-N[i].y * D[i]), p.z + (-N[i].z * D[i])) for i, p in enumerate(P)]
  
		# fill color array
		self.__tactile_point_cloud.colors = [ColorRGBA(self.__color_code[0], self.__color_code[1], self.__color_code[2], 1.0) for i in P]

		# pass along the remaining values from the contact state the data, to create the tactile point cloud
		self.__tactile_point_cloud.normals = self.contact_state.contact_normals

		# save if the tactile point cloud is empty
		self.__tactile_point_cloud.empty   = (len(self.contact_state.contact_positions) == 0)

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

	def __reconcile_contact_state(self, cs: ContactState) -> ContactState:
		"""due to Gazebo sometimes mirroring contact directions in simulation such that normals point in the wrong direction, the contact states vote on which direction is correct"""

		result: ContactState = cs

		# loop over all contact normals
		for i, n in enumerate(cs.contact_normals):

			# make an array of True or False, depending on the dot product of n and n1, n2, n3 etc. If the dot product is positive the vectors are pointing in the same direction and the value True is added to the list
			inlier_array: List[bool] = [True if gazebo.dot(n, ni) > 0 else False for ni in cs.contact_normals]
   
			# check if there are not Trues or Falses in the array
			is_inlier = True if inlier_array.count(True) >= inlier_array.count(False) else False
   
			# if there are more Trues than Falses, then n in an inlier and we continue
			if is_inlier:
				continue
			else:
				# flip the normal n and all other vectors in the contact state
				result.contact_normals[i]  = gazebo.prod(cs.contact_normals[i],  -1.0 )
				result.wrenches[i].force   = gazebo.prod(cs.wrenches[i].force,   -1.0 )
				result.wrenches[i].torque  = gazebo.prod(cs.wrenches[i].torque,  -1.0 )
				result.total_wrench.force  = gazebo.prod(cs.total_wrench.force,  -1.0 )
				result.total_wrench.torque = gazebo.prod(cs.total_wrench.torque, -1.0 )
				
		return result