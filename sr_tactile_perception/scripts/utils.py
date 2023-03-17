#!/usr/bin/python3
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D, proj3d
from typing import List
import numpy as np
from math import sqrt, acos, pi

class Vector3:
	
	def mul(self, s):
		if isinstance(s,float):
			return Vector3(self.x * s, self.y * s, self.z * s )
		else:
			return self.x*s.x + self.y*s.y + self.z*s.z
 
	def normalize(self):
		length = np.sqrt(pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2))
		# print(f"{type(length)} {type(self.x)}")
		self.x = self.x / length if length != 0 else 0
		self.y = self.y / length if length != 0 else 0
		self.z = self.z / length if length != 0 else 0
		return Vector3(self.x, self.y, self.z)
 
	def length(self) -> float:
		return np.sqrt(pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2))
 
	def angle(self,v,deg:bool = True) -> float:
		angle = acos( (self.mul(v)) / ( self.length() * v.length() ))
		if deg:
			return angle* (180.0 / pi)
		else:
			return angle
 
	def sub(self,v):
		return Vector3(self.x - v.x, self.y - v.y, self.z - v.z)

	def __str__(self):
		return f"({self.x},{self.y},{self.z})"

	def __repr__(self):
		return f"({self.x},{self.y},{self.z})"

	def __init__(self,x,y,z):
		self.x = x
		self.y = y
		self.z = z
  
  
class Arrow3D(FancyArrowPatch):
	def __init__(self, start: Vector3, end: Vector3, length: float = 1.0, *args, **kwargs):
			FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
			x = (start.x, end.x)
			y = (start.y, end.y)
			z = (start.z, end.z)

			self._verts3d = x, y, z

	def draw(self, renderer):
		# Tuple[float, float, float]     Tuple[float, float, float]      Tuple[float, float, float]
		xs3d, ys3d, zs3d = self._verts3d
		
		xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
		self.set_positions((xs[0], ys[0]), (xs[1],ys[1]))
		FancyArrowPatch.draw(self, renderer)
  
	def do_3d_projection(self, renderer=None):
		xs3d, ys3d, zs3d = self._verts3d
		xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
		self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
		return np.min(zs)

class COLOR():
	def __init__(self, label: str, color_code: List):
		self.label = label
		self.color_code = color_code

class COLORS_RGBA:
	# the colors associated with each finger according to https://elifesciences.org/articles/15292
	# from left most finger to right most: magenta(246, 0, 248, 255), blue(90, 0, 255, 255), green(0, 248, 0, 255), yellow(243, 255, 0, 255) and red(255, 0, 0, 255) in HLS format
	# the colors are extracted from a picture in the paper using https://imagecolorpicker.com/
	MAGENTA = COLOR("magenta", [0.964705882,       0.0, 0.97254902, 1.0])  # (246.0 / 255.0 , 0.0 / 255.0, 248.0 / 255.0, 255.0 / 255.0)
	BLUE = COLOR("blue", [0.352941176,       0.0,        1.0, 1.0])  # (90.0  / 255.0 , 0.0   / 255.0, 255.0 / 255.0, 255.0 / 255.0 )
	GREEN = COLOR("green", [0.0, 0.97254902,       0.0, 1.0])  # (0.0   / 255.0 , 248.0 / 255.0, 0.0   / 255.0, 255.0 / 255.0 )
	YELLOW = COLOR("yellow", [0.952941176,        1.0,       0.0, 1.0])  # (243.0 / 255.0 , 255.0 / 255.0, 0.0   / 255.0, 255.0 / 255.0 )
	RED = COLOR("red", [1.0,        0.0,       0.0, 1.0])  # (255.0 / 255.0 , 0.0   / 255.0, 0.0   / 255.0, 255.0 / 255.0 )


def l2(v: Vector3):
	return np.sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2))
