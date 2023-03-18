#!/bin/usr/python3

from utils import Vector3
from math import sqrt

FILE_PATHS_DATA = ["../data/stat_2023317_19544.json", "../data/stat_2023317_19952.json", "../data/stat_2023317_191745.json"]
# FILE_PATHS_DATA = ["data/stat_202337_10553.json", "data/stat_202337_10315.json", "data/stat_202337_105044.json"]

AXIS_LIMITS = {
	"x_lim": [
		0.09750656862858505,
		0.09122500964685813
	],
	"y_lim": [
		-0.3425221816775533,
		-0.32042612236717805
	],
	"z_lim": [
		0.9673374137539207,
		1.033989282610214
	]
}

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
EDGE_REF_VEC = (Vector3(1.0/sqrt(2), -1.0/sqrt(2), 0.0), Vector3(-1.0/sqrt(2), -1.0/sqrt(2), 0.0), Vector3(0.0, 1.0, 0.0))
SPHERE_REF_VEC = Vector3(0.0, 1.0, 0.0)
SPHERE_CENTER = Vector3(0.098090, -0.292110, 0.991616)

NUM_OF_HIS_BINS = 6