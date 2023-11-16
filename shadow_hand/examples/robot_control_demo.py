#!/usr/bin/env python3

import rospkg
import rospy
from ros_utils_py.utils import keep_alive
from ros_utils_py.geometry import geometry
from ros_utils_py.log import Logger
import time
import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.arm_finder import ArmFinder
from sr_utilities.hand_finder import HandFinder
import math as m
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from sensor_msgs.msg import JointState
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface


def get_model_pose(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state = get_model_state(model_name, 'world')
        return model_state.pose
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))




def main() -> None:
    log = Logger()

    # TODO : 
    # 	1) [x] perform planned motion
    # 	2) [x] perform planned motion + gravity
    # 	3) [x] place two boxes into world
    # 	4) [ ] build plan to pick up box 1
    # 	5) [ ] build plan to place box 1 onto box 2
    # 	6) [ ] complete
    #   ----- pain threshold -----
    # 	7) attempt to find digit tactile plugin and mesh for gazebo.

    # x y z r p y
    target_pose: PoseStamped = geometry.mk_pose_stamped(x=0.5, y=0.5, z=0.5, qx=1.0, qy=0.0, qz=0.0, w=m.pi)

    # waiting period for robot hand to start up...
    waiting_time: int = 10  # s
    rospy.loginfo(f"waiting {waiting_time} for hand to start...")
    time.sleep(waiting_time)

    cube_names = ["cube_1", "cube_2"]
    cube_poses = [get_model_pose(cube_names[0]), get_model_pose(cube_names[1])]

    for i, p in enumerate(cube_poses):
        log.success("Pose of {} in the world frame:{}".format(cube_names[i], p))

    # rospy.init_node("robot_commander_examples", anonymous=True)
    arm_commander = SrArmCommander(name="right_arm", set_ground=True)
    hand_finder = HandFinder()

    # hand finder - gets data from the found hand(s)
    hand_finder: HandFinder = HandFinder()

    # hand config object
    hand_parameters = hand_finder.get_hand_parameters()

    # serial numbers for the hands found
    hand_serial_numbers: list = list(
        hand_parameters.mapping.keys())

    # the 0'th hand serial number (['1234', '0']) '0' is here a dummy and '1234' is the serial for the simulated hand
    hand_serial: str = hand_serial_numbers[0]

    # get a hand commander, which can communicate with the hand specified by the inputs
    hand_commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial)
    log.success("----------------------------")
    tp: PoseStamped = geometry.pose2poseStamped(cube_poses[0])
    log.success(tp)

    # fk
    # hand_commander.get_end_effector_pose_from_state()

    # get world frame

    # hand_pose = hand_commander.get_current_pose()

    arm_pose = arm_commander.get_current_pose(reference_frame="world")
    log.warn(arm_commander.get_pose_reference_frame())
    log.warn(arm_pose)
    log.warn(hand_commander)

    name = "right_arm_and_hand"
    mg = MoveGroupCommander(name)
    
    log.warn(mg.get_current_joint_values())
    log.warn(mg.get_current_state())
    mg.set_end_effector_link("rh_WRJ2")
    log.warn(mg.get_end_effector_link())
    log.warn(mg.set_pose_target(pose=target_pose))
    # mg.set_pose_target(pose=target_pose)

    # right_arm_and_hand

    # tp.pose.position.z += 1.3

    # WORLD_FRAME = TransformStamped()
    # WORLD_FRAME.transform.rotation = Quaternion(0.0,0.0,0.0,1.0)
    # WORLD_FRAME.transform.translation = Point(0.0,0.0,0.0)

    # log.warn( hand_pose )
    # log.warn(WORLD_FRAME)
    # palm_pose = hand_commander.get_current_pose( reference_frame = "world" )
    # log.warn(palm_pose)

    # log.success(tp)
    # arm_commander.plan_to_pose_target(pose=tp)
    # arm_commander.move_to_pose_value_target_unsafe(target_pose=target_pose)

    # rospy.logerr(hand_commander.get_current_state())

    # hand_commander.plan_to_named_target("open")
    # hand_commander.move_to_named_target("open")
    # hand_commander.plan_to_named_target("pack")
    # hand_commander.move_to_named_target("pack")

    keep_alive(rospy.get_name())

if __name__ == '__main__':
    try:
        rospy.init_node("robot_control_demo", anonymous=True)
        main()

    except rospy.ROSInterruptException:
        pass