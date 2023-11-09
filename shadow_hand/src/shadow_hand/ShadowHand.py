#!/usr/bin/env python3

from ros_utils_py.log import Logger
import time
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
from typing import List, Dict, Optional, Union
import rospy
from gazebo_msgs.msg import ContactState, ContactsState

from shadow_hand.ShadowFinger import ShadowFinger
from shadow_hand.ShadowWrist import ShadowWrist
from ros_utils_py.pc_utils import PointCloudUtils as pcu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from ros_utils_py.gazebo import gazebo
import rosnode

class ShadowHand:
	"""A wrapper class for interacting with the Shadow Dexterous Hand"""

	def __init__(self):

		# logger
		self.__log = Logger()

		# wait for rviz-/move_group node to be initiated such that the hand can be communicated with
		timeout = 10
		t = 0
		while not rosnode.get_node_names().count("/move_group"):
			rospy.sleep(1)
			self.__log.warn(f"Waiting for /move_group to be initiated such that a connection to the Shadow Dexterous Hand can be made... Timeout {t}/{timeout} s...")
			t += 1
			if t >= timeout:
				self.__log.error("Failed to find /move_group and could therefore not connect to Shadow Dexterous Hand...")

		# has the biotac flag been set in launch file
		self.__is_biotac_sim_live = rospy.get_param('/biotac_sim')

		if self.__is_biotac_sim_live:
			self.__log.success("biotac_sim is live and ready for use...") 
		else:
			self.__log.warn("biotac_sim is NOT live...")

		self.__log.success("Successfully connected to Shadow Dexterous Hand...")

		# hand finder - gets data from the found hand(s)
		self.__hand_finder: HandFinder = HandFinder()

		# hand config object
		self.__hand_parameters = self.__hand_finder.get_hand_parameters()

		# serial numbers for the hands found
		self.__hand_serial_numbers: list = list(
			self.__hand_parameters.mapping.keys())

		# the 0'th hand serial number (['1234', '0']) '0' is here a dummy and '1234' is the serial for the simulated hand
		self.__hand_serial: str = self.__hand_serial_numbers[0]

		# get a hand commander, which can communicate with the hand specified by the inputs
		self.__hand_commander = SrHandCommander(hand_parameters=self.__hand_parameters, hand_serial=self.__hand_serial)

		# in the hand config object we get the prefix (i.e. chirality) (either 'rh' or 'lh') depending on the type of hand with the serial number
		self.__hand_chirality = self.__hand_parameters.mapping[self.__hand_serial]

		# get all the joint names in the hand with the hand prefix found above (e.g. 'rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', ...)
		self.__joints = self.__hand_finder.get_hand_joints()[self.__hand_chirality]

		# update frequency
		self.__update_frequency = 2.0

		# define fingers
		self.__thumb_finger = ShadowFinger(finger_type=ShadowFinger.FINGERS_NAMES.THUMB_FINGER, hc=self.__hand_commander)
		self.__index_finger = ShadowFinger(finger_type=ShadowFinger.FINGERS_NAMES.INDEX_FINGER, hc=self.__hand_commander)
		self.__middle_finger = ShadowFinger(finger_type=ShadowFinger.FINGERS_NAMES.MIDDLE_FINGER, hc=self.__hand_commander)
		self.__ring_finger = ShadowFinger(finger_type=ShadowFinger.FINGERS_NAMES.RING_FINGER, hc=self.__hand_commander)
		self.__little_finger  = ShadowFinger(finger_type=ShadowFinger.FINGERS_NAMES.LITTLE_FINGER, hc=self.__hand_commander)

		# define wrist
		self.__wrist = ShadowWrist(hc=self.__hand_commander,chirality=self.__hand_chirality)
  
		# tactile point cloud
		self.__tactile_point_cloud: pcu.PointCloud = pcu.PointCloud()

		# start recording tactile data
		self.__is_recording: bool = False
		if self.__is_biotac_sim_live:
			self.__sub_record_tac = rospy.Subscriber(f"/contacts/{self.__hand_chirality}_ff/distal", ContactsState, callback=self.__record_tac, queue_size=1)
			self.__save_path_record_tac = ""
		self.__recorded_tactile_point_cloud: pcu.PointCloud = pcu.PointCloud()
  
		self.__log.success("Successfully set up connection to Shadow Dexterous Hand...")

	@property
	def hand_commander(self):
		return self.__hand_commander

	@property
	def update_frequency(self) -> float:
		return self.__update_frequency

	@update_frequency.setter
	def update_frequency(self, new_update_frequency) -> None:
		self.__update_frequency = new_update_frequency
		for f in self.fingers:
			f.update_freq = new_update_frequency

	@property
	def thumb_finger(self) -> ShadowFinger:
		"""gets the hand's thumb as a ShadowFinger"""
		return self.__thumb_finger

	@property
	def index_finger(self) -> ShadowFinger:
		"""gets the hand's index finger as a ShadowFinger"""
		return self.__index_finger

	@property
	def middle_finger(self) -> ShadowFinger:
		"""gets the hand's middle finger as a ShadowFinger"""
		return self.__middle_finger

	@property
	def ring_finger(self) -> ShadowFinger:
		"""gets the hand's ring finger as a ShadowFinger"""
		return self.__ring_finger

	@property
	def little_finger(self) -> ShadowFinger:
		"""gets the hand's little finger as a ShadowFinger"""
		return self.__little_finger

	@property
	def hand_chirality(self) -> str:
		"""gets the hand's chirality (e.g. lh or rh)"""
		return self.__hand_chirality

	@property
	def joints_names(self) -> List[str]:
		"""get the joints of the shadow hand"""
		return self.__joints

	@property
	def fingers(self) -> List[ShadowFinger]:
		"""gets the ShadowFingers for this ShadowHand"""
		return [self.__thumb_finger, self.__index_finger, self.__middle_finger, self.__ring_finger, self.__little_finger]

	@property
	def q(self) -> Dict[str, float]:
		"""returns the angle for each joint in the hand in radians e.g. {'rh_FFJ1': 0.20, 'rh_FFJ2': 0.29, 'rh_FFJ3': 0.38, ... """
		return self.__hand_commander.get_joints_position()

	@property
	def dq(self) -> Dict[str, float]:
		"""returns the angular velocity for each joint in radians/s e.g. {'rh_FFJ1': 0.20, 'rh_FFJ2': 0.29, 'rh_FFJ3': 0.38, ... """
		return self.__hand_commander.get_joints_velocity()

	@property
	def tau(self) -> Dict[str,float]:
		"""returns the torque for each joint in Nm e.g. {'rh_FFJ1': 0.20, 'rh_FFJ2': 0.29, 'rh_FFJ3': 0.38, ... """
		return self.__hand_commander.get_joints_effort()

	@property
	def contact_states(self) -> Dict[ShadowFinger, Optional[ContactState]]:
		"""get a dictionary of all fingers' names and their contact states"""
		return {
			self.__thumb_finger  : self.__thumb_finger.contact_state,
			self.__index_finger  : self.__index_finger.contact_state,
			self.__middle_finger : self.__middle_finger.contact_state,
			self.__ring_finger   : self.__ring_finger.contact_state,
			self.__little_finger : self.__little_finger.contact_state
		}

	@property
	def tac_type(self) -> List[str]:
		"""get a list of the tactile types of the hand as used on every finger in order: [thumb_finger, index_finger, middle_finger, ring_finger, little_finger]"""
		return [f.tac_type for f in self.fingers]

	@property
	def wrist(self) -> ShadowWrist:
		"""get the shadow hand's wrist object ShadowWrist"""
		return self.__wrist

	@property
	def is_in_contact(self) -> bool:
		"""get if any of the shadow hand's fingers are in contact"""
		return any(f.is_in_contact for f in self.fingers)

	@property
	def is_biotac_sim_live(self) -> bool:
		"""get if the biotac_sim tactile library is live"""
		return self.__is_biotac_sim_live

	@property
	def tactile_point_cloud(self) -> pcu.PointCloud:
		"""a point cloud made from the tactile data from all fingers"""
		self.__tactile_point_cloud = pcu.PointCloud.empty()
		for f in self.fingers:
			self.__tactile_point_cloud.extend(f.tactile_point_cloud)
		return self.__tactile_point_cloud
		# return gazebo.combine_point_clouds(*[f.tactile_point_cloud for f in self.fingers])

	@property
	def max_force(self) -> Dict[ShadowFinger,List[float]]:
		return {
			self.__thumb_finger  : self.__thumb_finger.max_force ,
			self.__index_finger  : self.__index_finger.max_force ,
			self.__middle_finger : self.__middle_finger.max_force ,
			self.__ring_finger   : self.__ring_finger.max_force ,
			self.__little_finger : self.__little_finger.max_force
		}

	@max_force.setter
	def max_force(self, new_max_forces: Dict[ShadowFinger, List[Optional[float]]]) -> None:
		"""the maximum force the hand is allowed to exsert"""
		for f in new_max_forces:
			f.max_force = new_max_forces[f]
		return

	def record_start(self) -> None:
		"""when this function is called with True, the hand will start recording tactile data, and when called with false, the recording will stop"""
		self.__is_recording = True

	def record_stop(self,save_path:str):
		# self.__recorded_tactile_point_cloud.to_file(save_path)
		self.__save_path_record_tac = save_path
		self.__is_recording = False

	def __record_tac(self, data: Optional[ContactsState]) -> None:
		# rospy.sleep(0.05)
		"""runs every ff finger publish"""
		# the hand is recording
		if self.__is_recording:
			self.__recorded_tactile_point_cloud.extend(self.tactile_point_cloud)
		# we are done recording and want to save the data
		elif len(self.__recorded_tactile_point_cloud) != 0 and not self.__is_recording:
			print("saving pcd ------------------------------------------")
			self.__recorded_tactile_point_cloud.write_pcd(self.__save_path_record_tac)
   
			self.__recorded_tactile_point_cloud = pcu.PointCloud.empty()
		# we have not started to record yet or have already done so
		else:
			return

	def set_q(self, des_state: Dict[Union[ShadowFinger, ShadowWrist], List[Optional[float]]], interpolation_time: int = 1,block: bool = False) -> None:
		"""sets all joint configurations as specified in the dictionary. One such example can be seen below

			hand_q = {
				sh.thumb_finger  : [0.0, 0.0, π/2.0],
				sh.index_finger  : [0.0, 0.0, π/2.0],
				sh.middle_finger : [0.0, 0.0, π/2.0],
				sh.ring_finger   : [0.0, 0.0, π/2.0],
				sh.little_finger : [0.0, 0.0, π/2.0],
				sh.wrist         : [0.0, 0.0]
			}
			if you want certain joints unchanged, simply leave a None. The interpolation time is in seconds and block is rather or not the operation should be blocking for the program.
		Returns:
			bool: if the setting succeeded
		"""
		joint_trajectory = self.__make_jt(des_state, interpolation_time=interpolation_time)
		self.__hand_commander.run_joint_trajectory_unsafe(joint_trajectory,wait=block)
		return

		# joints_states = {'rh_FFJ1': 90, 'rh_FFJ2': 90, 'rh_FFJ3': 90, 'rh_FFJ4': 0.0,
		#            'rh_MFJ1': 90, 'rh_MFJ2': 90, 'rh_MFJ3': 90, 'rh_MFJ4': 0.0,
		#            'rh_RFJ1': 90, 'rh_RFJ2': 90, 'rh_RFJ3': 90, 'rh_RFJ4': 0.0,
		#            'rh_LFJ1': 90, 'rh_LFJ2': 90, 'rh_LFJ3': 90, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
		#            'rh_THJ1': 40, 'rh_THJ2': 35, 'rh_THJ3': 0.0, 'rh_THJ4': 65, 'rh_THJ5': 15,
		#            'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
		# 'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5'

	def __make_jt(self,state: Dict[Union[ShadowFinger,ShadowWrist],List], interpolation_time: int = 3) -> JointTrajectory:
		joint_values_dict = self.__get_q_as_dict(state)
	
		jt = JointTrajectory()
		all_joint_names = [jn for jn, jv in joint_values_dict.items()] 
		all_joint_values = [jv for jn, jv in joint_values_dict.items()]
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

	def __get_q_as_dict(self,des_state: Dict[Union[ShadowFinger,ShadowWrist],List[Optional[float]]]) -> Dict[str,float]:
		# make des_state valid
		valid_des_state: Dict[ShadowFinger, List[float]] = {}
		for f in des_state:
			valid_des_state[f] = f.make_valid_q(des_state[f])

		des_joint_state = {}
		for finger in valid_des_state.keys():  # index_finger, middle_finger, ...
			for i, jn in enumerate(finger.joint_names):  # 0 rh_FFJ1, 1 rh_FFJ2, ...
				des_joint_state[jn] = valid_des_state[finger][i]
		return des_joint_state