#!/usr/bin/python3
import time
import rospy

rospy.init_node("hand_finder_example", anonymous=True)

while (True):
    time.sleep(1)
    rospy.logerr("echo...")


# # Using the HandFinder
# hand_finder = HandFinder()
# hand_parameters = hand_finder.get_hand_parameters()
# hand_serial = hand_parameters.mapping.keys()

# rospy.logerr(hand_parameters.mapping)

# # If name is not provided, it will set "right_hand" or "left_hand" by default, depending on the hand.
# # hand_commander = SrHandCommander(name = "rh_first_finger",
# #                                  hand_parameters=hand_parameters,
# #                                  hand_serial=hand_serial)

# # Alternatively you can launch the hand directly
# hand_commander = SrHandCommander(name = "right_hand", prefix = "rh")

# hand_joints_effort = hand_commander.get_joints_effort()
# rospy.logerr("Hand joints effort \n " + str(hand_joints_effort) + "\n")

# tactile_state = hand_commander.get_tactile_state()
# rospy.logerr("Hand tactile state\n" + str(tactile_state) + "\n")

# tactile_type = hand_commander.get_tactile_type()

# rospy.logerr(tactile_type)

# # print("Hand tactile type\n" + tactile_type + "\n")
