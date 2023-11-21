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
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, Pose
from sensor_msgs.msg import JointState
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import moveit_commander
import copy
import moveit_msgs.msg
from shadow_hand import ShadowHand
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import (
    RobotTrajectory,
    Grasp,
    PlaceLocation,
    Constraints,
    RobotState,
)
import pickle
import os

def get_model_pose(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state = get_model_state(model_name, 'world')
        return model_state.pose
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def print_group_info(group_name):

    _log = Logger()

    # Create a MoveGroupCommander instance for the group
    group = robot.get_group(group_name)

    # Print information about the group
    _log.warn("   Group: {}".format(group_name))
    try:
        _log.warn("   End Effector Link: {}".format(group.get_end_effector_link()))
    except:
        _log.warn("   No Effector Link Found")
    
    try:
        _log.warn("   Current Joint Values: {}".format(group.get_current_joint_values()))
    except:
        _log.warn("   No Joint Values Found")

    try:
        _log.warn("   Current Pose: {}".format(group.get_current_pose()))
    except:
        _log.warn("   No Current Pose Found")

def save_robot_traj(rt: RobotTrajectory, file_path:str) -> None:
    # if file does not exist, create one
    if not os.path.exists(file_path):
        f = open(file_path,"w")
        f.close()


    with open(file_path, 'wb') as fp:
        pickle.dump(rt, fp)

def load_robot_traj(file_path:str) -> RobotTrajectory:
    plan = None
    with open(file_path, 'rb') as file_open:
        plan = pickle.load(file_open)[1]
    return plan

def main() -> None:
    log = Logger()
    # waiting period for robot hand to start up...
    waiting_time: int = 10  # s
    rospy.loginfo(f"waiting {waiting_time} for hand to start...")
    time.sleep(waiting_time)

    # TODO : 
    # 	1) [x] perform planned motion
    # 	2) [x] perform planned motion + gravity
    # 	3) [x] place two boxes into world
    # 	4) [ ] build plan to pick up box 1
    # 	5) [ ] build plan to place box 1 onto box 2
    # 	6) [ ] complete
    #   ----- pain threshold -----
    # 	7) attempt to find digit tactile plugin and mesh for gazebo.

    name = "right_arm_and_manipulator"

    mg = MoveGroupCommander(name)

    # x y z r p y
    d_grasp = 0.1
    # d_grasp = 0.08
    d_place = 0.5

    flat_orientation = geometry.euler2quaternion(m.pi/2.0, 0.0, m.pi)

    home: Pose = Pose(
        position = copy.deepcopy(mg.get_current_pose().pose.position),
        orientation = flat_orientation
    )

    box_1_pose: Pose = Pose(position = get_model_pose("cube_1").position, orientation = flat_orientation)
    box_2_pose: Pose = Pose(position = get_model_pose("cube_2").position, orientation = flat_orientation)

    box_1_grasp_pose: Pose = box_1_pose
    box_1_grasp_pose.position.z += d_grasp
    box_1_grasp_pose.position.y -= 0.05

    box_2_grasp_pose: Pose = box_2_pose
    box_2_grasp_pose.position.z += d_grasp

    middle_pose: Pose = Pose(
        position = Point(
            x = box_1_grasp_pose.position.x + box_2_pose.position.x / 2.0,
            y = box_1_grasp_pose.position.y,
            z = box_1_grasp_pose.position.z + d_place),
        orientation = flat_orientation
        )

    box_1_hover_pose: Pose = Pose(
        position = Point(
            x = box_1_grasp_pose.position.x,
            y = box_1_grasp_pose.position.y,
            z = box_1_grasp_pose.position.z + d_place),
        orientation = flat_orientation
    )
    box_2_hover_pose: Pose = Pose(
        position = Point(
            x = box_2_grasp_pose.position.x,
            y = box_2_grasp_pose.position.y,
            z = box_2_grasp_pose.position.z + d_place),
        orientation = flat_orientation
    )

    way_to_box_1 = [
        # home,
        middle_pose,
        box_1_hover_pose,
        box_1_grasp_pose
    ]
    way_to_box_2 = [
        box_1_hover_pose,
        middle_pose,
        box_2_hover_pose,
        box_2_grasp_pose
    ]

    way_to_return = [
        box_2_hover_pose,
        home
    ]
    
    (plan_1, fraction_1) = mg.compute_cartesian_path(waypoints=way_to_box_1, eef_step=0.001, jump_threshold=0.0)
    (plan_2, fraction_2) = mg.compute_cartesian_path(waypoints=way_to_box_2, eef_step=0.001, jump_threshold=0.0)
    (plan_3, fraction_3) = mg.compute_cartesian_path(waypoints=way_to_return, eef_step=0.001, jump_threshold=0.0)

    log.warn(f"fractions 1: {fraction_1}, 2: {fraction_2}, 3: {fraction_3}")

    # mg.compute_cartesian_path(waypoints=waypoints)
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    display_trajectory.trajectory_start = mg.get_current_state()
    display_trajectory.trajectory.append(plan_1)
    display_trajectory_publisher.publish(display_trajectory)

    sh = ShadowHand()

    q_thumb_max = 1.22 # rad, 70 in deg
    
    open = {
        sh.index_finger:  [0.0, 0.0, 0.0],
        sh.middle_finger: [0.0, 0.0, 0.0],
        sh.ring_finger:   [0.0, 0.0, 0.0],
        sh.little_finger: [0.0, 0.0, 0.0],
        sh.thumb_finger:  [0.0, 0.0, 0.0, q_thumb_max]
    }

    close_angle = m.pi/5.0

    close = {
        sh.index_finger:  [close_angle, close_angle, close_angle],
        sh.middle_finger: [close_angle, close_angle, close_angle],
        sh.ring_finger:   [close_angle, close_angle, close_angle],
        sh.little_finger: [close_angle, close_angle, close_angle],
        sh.thumb_finger:  [close_angle, close_angle, close_angle]
    }

    plan_1_path = "/home/user/projects/shadow_robot/base/src/in_hand_pose_estimation/shadow_hand/experiments/joint_trajectories/plan_1.pickle"

    save_robot_traj(plan_1,plan_1_path)
    # plan_1 = load_robot_traj(plan_1_path)
    # execute motion
    mg.execute(plan_1, wait=True)
    sh.set_q(open)
    sh.hand_commander.attach_object("cube_1")
    time.sleep(2)
    sh.set_q(close,interpolation_time=3)



    # show movegroups ##################################################
    # global robot
    # robot = RobotCommander()

    # # Get the list of all groups
    # all_groups = robot.get_group_names()

    # log.warn("List of MoveIt groups:")
    # for group in all_groups:
    #     log.warn("- {}".format(group))
    #     print_group_info(group)

    # execute movements ##################################################

    while (mg.get_current_pose() != home):
        time.sleep(0.5)
        log.success(f"   current pose {mg.get_current_pose().pose.position}")

    # for jn in mg.get_joints():
    #     try: 
    #         log.warn(mg.set_end_effector_link(jn))
    #         log.success(f"Set {jn} as the end effector")
    #     except:
    #         log.error(f"Attempted to set {jn} as the end effector....but failed")

    # log.warn(mg.set_end_effector_link("rh_palm"))
    # mg.robot.get_group("panda_arm").set_end_effector_link("panda_gripper_center"
    # mg.set_end_effector_link("rh_WRJ2")
    # log.warn("end effector link =" + str(mg.get_end_effector_link()))
    # log.warn("get current pose ="  + str(mg.get_current_pose()))
    # log.warn(mg.set_pose_target(pose=target_pose))
    


    # scene = moveit_commander.PlanningSceneInterface()
    # box_pose = PoseStamped()
    # box_pose.header.frame_id = "world"
    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.y = 5
    # box_name = "box"
    # scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

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