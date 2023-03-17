#!/bin/usr/python3

from typing import Dict, List
from utils import Vector3
import json
from utils import COLORS_RGBA, COLOR

class DataHandler:

	def __init__(self, file_path):
		self.__file_path = file_path
		self.__data_json = {}
		with open(file_path, "r") as __f:
			self.__data_json = json.load(__f)
		# print(self.__data_json)
		self.__experiment_config = DataHandler.__ExperimentConfig(self.__data_json["experiment_config"])
		self.__time_states = [DataHandler.TimeState(T) for T in self.__data_json.items() if (T[0] != "experiment_config")]

	@property
	def time_states(self):
		return self.__time_states
	@property
	def experiment_config(self):
		return self.__experiment_config

	@property
	def file_path(self):
		return self.__file_path


	class __ExperimentConfig:
		def __init__(self, experiment_config: Dict):
			self.__experiment_config_dict = experiment_config
			self.__num_of_dp = experiment_config["num_of_dp"]
			self.__dt = experiment_config["dt"]
			self.__prop = experiment_config["prop_name"]

		@property
		def num_of_dp(self):
			return self.__num_of_dp

		@property
		def dt(self):
			return self.__dt

		@property
		def prop(self):
			return self.__prop

	class Finger:

		__label_enum = {
			"rh_TH": COLORS_RGBA.MAGENTA,
			"rh_FF": COLORS_RGBA.BLUE,
			"rh_MF": COLORS_RGBA.GREEN,
			"rh_RF": COLORS_RGBA.YELLOW,
			"rh_LF": COLORS_RGBA.RED
		}

		__label_verbose_enum = {
			"rh_TH": "thumb_finger",
			"rh_FF": "first_finger",
			"rh_MF": "middle_finger",
			"rh_RF": "ring_finger",
			"rh_LF": "little_finger"
		}

		def __init__(self, finger: Dict):
			self.__name = list(finger.keys())[0]
			self.__finger_dict = finger[self.__name]
			self.__number_of_contacts = self.__finger_dict["nc"]
   
			# make data formats of normals, forces, torques, and contact points from [p1, p2, p3] into (list_of_x, list_of_y, list_of_z)
    
			l_normals = list(self.__finger_dict["normals"].values())
			self.__normals = ([n[0] for n in l_normals], [n[1] for n in l_normals], [n[2] for n in l_normals])
   
			l_forces = list(self.__finger_dict["forces"].values())
			self.__forces = ([n[0] for n in l_forces], [n[1] for n in l_forces], [n[2] for n in l_forces])
   
			l_torques = list(self.__finger_dict["torques"].values())
			self.__torques = ([n[0] for n in l_torques], [n[1] for n in l_torques], [n[2] for n in l_torques])
   
			l_contact_points = list(self.__finger_dict["contact_points"].values())
			self.__contact_points = ([n[0] for n in l_contact_points], [n[1] for n in l_contact_points], [n[2] for n in l_contact_points])

			self.__normals_vec = [Vector3(*v) for v in l_normals]
			self.__forces_vec  = [Vector3(*v) for v in l_forces]
			self.__torques_vec = [Vector3(*v) for v in l_torques]
			self.__contact_points_vec = [Vector3(*v) for v in l_contact_points]

			self.__color: COLOR = DataHandler.Finger.__label_enum[self.__name]

			self.__name_verbose: str = DataHandler.Finger.__label_verbose_enum[self.__name]

		@property
		def name(self):
			return self.__name

		@property
		def name_verbose(self):
			return self.__name_verbose

		@property
		def number_of_contacts(self):
			return self.__number_of_contacts

		@property
		def normals(self):
			return self.__normals

		@property
		def normals_vec(self):
			return self.__normals_vec

		@property
		def forces(self):
			return self.__forces

		@property
		def forces_vec(self):
			return self.__forces_vec

		@property
		def torques(self):
			return self.__torques

		@property
		def torques_vec(self):
			return self.__torques_vec

		@property
		def contact_points(self):
			return self.__contact_points

		@property
		def contact_points_vec(self):
			return self.__contact_points_vec

		@property
		def color(self):
			return self.__color

	class TimeState:
		def __init__(self, time_state: Dict):
			self.__time_state_dict = time_state
			self.__time_step = time_state[0][1:]
			#  create an array of the fingers dicts
			dicts = self.__time_state_dict[1]
			self.__fingers = [DataHandler.Finger({label : dict}) for label, dict in dicts.items()]

		@property
		def time_step(self):
			return self.__time_step

		@property
		def fingers(self):
			return self.__fingers

