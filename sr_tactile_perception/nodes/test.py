#!/usr/bin/env python3
#
# Copyright 2020 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

# from __future__ import absolute_import
# from builtins import input
# import rospy
# import rospkg
# from sr_run_trajectories.run_trajectories import SrRunTrajectories


# if __name__ == "__main__":
#     rospy.init_node('run_hand_poses')
#     hand_type = rospy.get_param('~hand_type', 'hand_e')
#     biotac = rospy.get_param('~biotac', False)

#     # TODO: Extend with more hand types if necessary
#     if 'hand_e' == hand_type:
#         poses_yaml_file_name = 'hand_poses_to_test_hand_e'
#         if biotac:
#             poses_yaml_file_name += '_biotac'
#     else:
#         raise ValueError("Unknown hand type!")

#     trajectories_file_path = rospkg.RosPack().get_path('sr_pose_tests') + '/config/{}.yaml'.format(poses_yaml_file_name)
#     srt = SrRunTrajectories(trajectories_file_path, arm=False)

#     for pose in srt._hand_trajectories:
#         if 'open' == pose:
#             continue
#         input("About to go to pose {}. Press [RETURN] to execute...".format(pose))
#         srt.run_trajectory('hand', pose)
#         input("Press [RETURN] to go back to open pose")
#         srt.run_trajectory('hand', 'open')
#     rospy.loginfo("All poses have been tested. Exiting.")
