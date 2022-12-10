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








# #include "ros/node_handle.h"
# #include <ros/ros.h>
# #include <iostream>
# #include <sr_hand/hand_commander.hpp>
# #include <sr_utilities/sr_hand_finder.hpp>
# // #include <sr_robot_msgs.h>
# #include <sr_robot_msgs/joint.h>

# int main (int argc, char **argv)
# {
#     // init node
#     ros::init(argc, argv, "sr_test");

#     ros::NodeHandle nh;

#     auto hand_finder = shadow_robot::SrHandFinder();
#     auto hand_parameters = hand_finder.get_hand_parameters();
#     auto hand_serial = hand_parameters.mapping_.begin()->first; // the first key

#     // obs if does not work, try "rh/right_hand" instead
#     auto hand_commander = shadowrobot::HandCommander("right_hand");

#     // auto hand_joint_effort = hand_commander

#     // load hand controller 
#     // auto hc = shadowrobot::HandCommander();
#     // auto a = hc.get_all_joints();

#     // std::vector<sr_robot_msgs::joint> joints;
#     // for (auto& j : joints)
#     //     j.joint_target = 0.0;

#     ros::Rate rate(1); // ROS Rate at 5Hz
    
#     while ( ros::ok() )
# 	{
#         ROS_ERROR(a[0].c_str());
#         ROS_ERROR("Hello");
#         rate.sleep();
#     }
# }

# /*
# stuff to remember:
#  + #include "sr_tactile_sensors/sr_virtual_tactile_sensor.hpp"

# */