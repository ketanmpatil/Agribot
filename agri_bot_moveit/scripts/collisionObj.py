#! /usr/bin/env python3

import rospy
from moveit_msgs.msg import Grasp, CollisionObject, ExecuteTrajectoryAction
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
import moveit_commander
import actionlib

def pick(group):
    grasps = Grasp()
    p = group.get_current_pose().pose
    grasps.grasp_pose.header.frame_id = "ebot_base"
    grasps.grasp_pose.pose.orientation.x = p.orientation.x
    grasps.grasp_pose.pose.orientation.y = p.orientation.y
    grasps.grasp_pose.pose.orientation.z = p.orientation.z
    grasps.grasp_pose.pose.orientation.w = p.orientation.w
    grasps.grasp_pose.pose.position.x = 0.4
    grasps.grasp_pose.pose.position.y = 0
    grasps.grasp_pose.pose.position.z = 1


    # grasps.pre_grasp_approach.direction.header.frame_id = 'ebot_base'
    # grasps.pre_grasp_approach.direction.vector.x = 1

    # grasps.post_grasp_retreat.direction.header.frame_id = "ebot_base"
    # grasps.post_grasp_retreat.direction.vector.z = 1.0

    # go_to_predefined('gripper_planning_group', 'open')
    # print(grasps)

    group.pick('sphere',grasps)

    # go_to_predefined('gripper_planning_group', 'close')

    


def go_to_predefined(planning_group, arg_pose_name):
    '''
    Function defined to move the arm to a predefined pose.
    It takes 2 argument, 
    planning_group: Name of Planning Group
    arg_pose_name: name of predefined pose
    '''
    group = moveit_commander.MoveGroupCommander(planning_group)
    
    group.allow_looking(True)
    group.allow_replanning(True)

    group.set_goal_tolerance(0.05)



    exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
    exectute_trajectory_client.wait_for_server()

    rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
    group.set_named_target(arg_pose_name)
    group.go(wait = True)
    rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    

def add_scene_obj(x,y,z):
    ''' Add the scene objects, for the given transformations.
        takes the mesh from local folders.
    '''

def addCollisionObject(scene):
    collision_obj = CollisionObject()

    collision_obj.header.frame_id = 'ebot_base'
    collision_obj.id = 'sphere'
    collision_obj.pose.orientation.w=1

    obj = SolidPrimitive()
    obj.type = SolidPrimitive.SPHERE
    obj.dimensions = [0.02]
    

    collision_obj.primitives.append(obj)
    pose = PoseStamped()
    pose.pose.position.x = 0.7
    pose.pose.position.y = 0
    pose.pose.position.z = 1
    pose.pose.orientation.y = 1
    collision_obj.primitive_poses.append(pose.pose)
    collision_obj.operation = collision_obj.ADD
    # print(collision_obj)
    rospy.sleep(1)

    scene.add_object(collision_obj)

def add_scene_obj(scene,x,y,z):
    ''' Add the scene objects, for the given transformations.
        takes the mesh from local folders.
    '''

    rospy.sleep(1)

    scene = moveit_commander.PlanningSceneInterface(synchronous = True)
    box_pose = PoseStamped()
    box_pose.header.frame_id = "ebot_base"
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z # above the panda_hand frame
    box_name = "sphere"

    scene.add_mesh(box_name, box_pose, "/home/wolf/wolf_ws/src/eYRC-2021_Agribot/tomato_gazebo/models/tomato/meshes/tomato.obj", size=(0.7,0.7,0.7))

    rospy.loginfo('Added Object')


    

def main():

    rospy.init_node('obj', anonymous=True)
    rate = rospy.Rate(10)
    planning_scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm_planning_grp')
    # group.set_planning_time(45.0)
    # group.set_goal_tolerance(0.01)
    group.set_num_planning_attempts(5)
    addCollisionObject(planning_scene)
    # add_scene_obj(planning_scene)
    # go_to_predefined('gripper_planning_group', 'open')
    group.pick('sphere')
    # pick(group)
    

    
if __name__ == '__main__':
    main()