#!/bin/usr/python3

from utils import Vector3
from math import sqrt
import glob
from typing import Tuple, List
import json
import os

def get_date_num(path: str) -> str:
	return "".join(path.split("/")[-1].split(".")[0].split("_"))

def newest_data_set_path(prop_name: str) -> str:
	data_file_paths = glob.glob("../data/*")
	new_prop_num = 0
	new_prop_path = ""
	for path in data_file_paths:
		with open(path, "r") as f:
			# print(path)
			# in case the file is empty
			if os.path.getsize(path) == 0: continue
			# load json
			js = json.load(f)
			if "experiment_config" not in js:
				return ""
			if js["experiment_config"]["prop_name"] == prop_name and int(get_date_num(path)) > new_prop_num:
				new_prop_num = int(get_date_num(path))
				new_prop_path = path
	return new_prop_path

# FILE_PATHS_DATA = [newest_data_set_path("cube"), newest_data_set_path("edge"), newest_data_set_path("sphere"), newest_data_set_path("stanford_bunny")]

# AXIS_LIMITS = {
# 	"x_lim": [
# 		0.124796, 
# 		0.144796
# 	],
# 	"y_lim": [
# 		-0.291485, 
# 		-0.271485
# 	],
# 	"z_lim": [
# 		0.981616, 
# 		1.001616
# 	]
# }
extra_pad = 0.007
AXIS_LIMITS = {
	"x_lim": [
		0.09750656862858505 + extra_pad,
		0.09122500964685813 - extra_pad
	],
	"y_lim": [
		-0.3425221816775533 - extra_pad,
		-0.32042612236717805 + extra_pad
	],
	"z_lim": [
		0.9673374137539207 - extra_pad,
		1.033989282610214 + extra_pad
	]
}
# [0.124796, 0.144796]
# [-0.291485, -0.271485]
# [0.981616, 1.001616]

LEGEND_ANCHOR = (0.8, 0.6, 0.5, 0.5)
SPACE_SCALE = 25.0

X_LABEL = "x [m]"
Y_LABEL = "y [m]"
Z_LABEL = "z [m]"

FLAT_TEXT       = (0.23, 0.95, "FLAT")
EDGE_TEXT       = (0.50, 0.95, "EDGE")
SPHERE_TEXT     = (0.77, 0.95, "SPHERE")
NORMALS_TEXT    = (0.05, 0.70, "NORMALS")
FORCES_TEXT     = (0.05, 0.45, "FORCES")
TORQUES_TEXT    = (0.05, 0.20, "TORQUES")
TIME_STATE_TEXT = (0.05, 0.95)



VECTOR_SCALE = abs(AXIS_LIMITS["x_lim"][0] / SPACE_SCALE)  # just a test
MARKER_SIZE = 10
NUMBER_OF_PLOTS = 9

FLAT_REF_VEC = Vector3(0.0, 1.0, 0.0)
EDGE_REF_VEC = (
    Vector3( -1.0 * -1.0/sqrt(2) , -1.0 * -1.0/sqrt(2) , 0.0), 
    Vector3( -1.0 * 0.0          , -1.0 * -1.0         , 0.0),
    Vector3( -1.0 * 1.0/sqrt(2)  , -1.0 * -1.0/sqrt(2) , 0.0))
# EDGE_REF_VEC = (
#     Vector3(-1.0/sqrt(2), -1.0/sqrt(2), 0.0), 
#     Vector3(0.0, -1.0, 0.0),
#     Vector3(1.0/sqrt(2), -1.0/sqrt(2), 0.0))
SPHERE_REF_VEC = Vector3(0.0, 1.0, 0.0)
SPHERE_CENTER = Vector3(0.098090, -0.292110, 0.991616)

NUM_OF_HIS_BINS = 6

CUBE_POSE = ([0.134796, -0.281485, 0.991616], 0.01, 0.01, 0.01, 0.0, 0.0, 0.0)
