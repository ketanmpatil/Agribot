#! /usr/bin/env python3
from importlib.resources import path
from re import M
import rospy
import moveit_commander
from moveit_msgs.msg import RobotState, Constraints, PositionConstraint, BoundingVolume, OrientationConstraint
from std_msgs.msg import String

group = moveit_commander.MoveGroupCommander('arm_planning_grp')



if __name__ == "__main__":
    constraints(group.get_current_pose())

print(group.get_end_effector_link())
