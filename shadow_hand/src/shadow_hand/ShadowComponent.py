#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import Dict, List

class ShadowComponent(ABC):

	@abstractmethod
	def get_q(self) -> Dict[str,List]:
		"""get the current configuration q for each joint in radians e.g.

		{
			rh_WRJ1 : [pi / 2],
			rh_WRJ2 : [pi],
		}

		Returns:
			Dict[str,List]: the current configuration q for each joint in radians
		"""
		return {}

	@abstractmethod
	def set_q(self) -> bool:
		"""sets the configuration of the component to q in the order: tip_q, middle_q and base_q, or yaw_q, pitch_q e.g.

		for a finger: List[float]
		q = [q1, q2, q3], where each qi is in radians. The angles are read from tip to base of the finger.
  
		for a hand: Dict[str, List[float]]
		{
			"rh_TH" : [q1, q2, q3],
			"rh_FF" : [q1, q2, q3],
			"rh_MF" : [q1, q2, q3],
			"rh_RF" : [q1, q2, q3],
			"rh_LF" : [q1, q2, q3]
		}
		in a hand, not all fingers have the same number of joints, so the default
		behavior is to get an many joints as values provided and the remaining joints
		are just kept constant to their current value.

		for a wrist: List[float]
		q = [q1, q2, q3], is the same as for a finger, here the order is yaw and pitch
  
		Args:
			q (list): the q you want the component to have

		Returns:
			bool: did the set function succeed
		"""
		pass

	def get_name(self) -> str:
		
		return ""