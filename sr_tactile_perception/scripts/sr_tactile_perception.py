#!/usr/bin/env python3

import rospy
from ros_utils_py.utils import devprint, keep_alive
import time

def main() -> None:
	keep_alive(rospy.get_name())

if __name__ == '__main__':
	try:
		rospy.init_node("sr_tactile_perception", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
