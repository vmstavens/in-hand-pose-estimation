#!/usr/bin/env python3

from __future__ import absolute_import
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
import time

see_me = "##################################################################################################################"

rospy.init_node("test", anonymous=True)
rospy.logerr("testing1...")
hand_finder = HandFinder()
rospy.logerr("testing2...")

while (True):
    time.sleep(1.0)

    hand_parameters = hand_finder.get_hand_parameters()
    
    hand_serial = list(hand_parameters.mapping.keys())[0]
    rospy.loginfo(f"{hand_serial=} {see_me}")
    
    rospy.loginfo(f"{hand_parameters=} | {hand_serial=}")
    
    hand_commander = SrHandCommander(hand_parameters=hand_parameters,hand_serial=hand_serial)
    # rospy.logdebug(f"{hand_commander=}...")

    rospy.sleep(1.0)

    # rospy.logerr("Tactile type: ", hand_commander.get_tactile_type())
    # rospy.logerr("Tactile state: ", hand_commander.get_tactile_state())
    
    # hand_parameters = hand_finder.get_hand_parameters()
    # hand_serial = hand_parameters.mapping.keys()

    # rospy.logerr(hand_parameters.mapping)

    # # If name is not provided, it will set "right_hand" or "left_hand" by default, depending on the hand.
    # hand_commander = SrHandCommander(name = "rh_first_finger",
    #                                  hand_parameters=hand_parameters,
    #                                  hand_serial=hand_serial)

    # # # Alternatively you can launch the hand directly
    # # hand_commander = SrHandCommander(name = "right_hand", prefix = "rh")

    # hand_joints_effort = hand_commander.get_joints_effort()
    # rospy.logerr("Hand joints effort \n " + str(hand_joints_effort) + "\n")

    # tactile_state = hand_commander.get_tactile_state()
    # rospy.logerr("Hand tactile state\n" + str(tactile_state) + "\n")

    # tactile_type = hand_commander.get_tactile_type()

    # rospy.logerr(tactile_type)

    # rospy.logerr("Hand tactile type\n" + tactile_type + "\n")
