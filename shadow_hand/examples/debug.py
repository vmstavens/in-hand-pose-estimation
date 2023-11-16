#!/usr/bin/env python

import rospy
from moveit_commander import RobotCommander
from ros_utils_py.log import Logger
from ros_utils_py.utils import keep_alive
import time
from moveit_commander.move_group import MoveGroupCommander
def main():
    # Initialize the ROS node
    rospy.init_node('list_moveit_groups', anonymous=True)

    waiting_time: int = 10  # s
    rospy.loginfo(f"waiting {waiting_time} for hand to start...")
    time.sleep(waiting_time)

    # right_arm_and_hand

    log = Logger()

    # Create a RobotCommander instance
    robot = RobotCommander()

    group: MoveGroupCommander = robot.get_group("right_arm_and_hand")



    # group.

    log.success("group: " + str(group))

    # Get the list of all groups
    all_groups = robot.get_group_names()

    log.success("List of MoveIt groups:")
    for group in all_groups:
        log.success("- {}".format(group))

    keep_alive(rospy.get_name())


if __name__ == '__main__':
    main()
