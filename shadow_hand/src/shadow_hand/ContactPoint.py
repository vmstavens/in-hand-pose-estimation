#!/usr/bin/env python3

from typing import List
from geometry_msgs.msg import Vector3


class ContactPoint:
	
	def __init__(self, contact_position: Vector3, contact_normal: Vector3, wrench: Vector3, depth: float):
		self.contact_position = (contact_position)
		self.contact_normal = (contact_normal)
		self.wrench = (wrench)
		self.depth:float = depth