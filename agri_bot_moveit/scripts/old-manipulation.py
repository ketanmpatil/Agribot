#! /usr/bin/env python3

import tf2_ros
import sys
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import std_msgs
from std_msgs.msg import Bool, Int64, String


# import os
# os.path.join('/home/mustang/mustang_ws/src/eYRC-2021_Agribot/ebot_gazebo')
# from ebot_gazebo.msg import FeedbackAction

# INITIALIZATION OF COORDINATES 
x = 0
y = 0
z = 0
transformations = None

move = None
count = 0
direction = "l"
def nav_callback(data):
    global move
    move = data.data

def dir_callback(data):
    global direction
    direction = data.data

def per_callback(data):
    global count
    count = data.data

rospy.init_node('manipulation', anonymous=True) # ROS Node Initialization
commander = moveit_commander.roscpp_initialize(sys.argv) # Moveit Commander Initialization
robot = moveit_commander.RobotCommander() # Robot Commander


def end_eff_pose(x, y, z, planning_group):
    ''' Takes coordinates and name of the planning group as an input and commands the end to 
    that position. '''
    move_group = moveit_commander.MoveGroupCommander(planning_group)

    move_group.allow_replanning(True)

    # move_group.set_goal_tolerance(0.05)

    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.header.frame_id = 'ebot_base'

    # DEFAULT ORIENTATION, CALCULATED FROM MANUAL METHODS 

    roll = -3.0542428672178215
    pitch = 9.25869263155539e-05
    yaw = -3.03682688092606

    # TO CONVERT EULER COORDINATES TO QUATERNION 
    quaternion = tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)

    pose_goal.pose.orientation.y = 1

    pose_goal.pose.position.x = x
    pose_goal.pose.position.y = y -0.27 # -0.27 IS THE OFFSET FROM ENDPOSE TO GRIPPERS
    pose_goal.pose.position.z = z

    try:
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()
        move_group.execute(plan)
    except:
        print('No Solution Found')

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

    # group.set_goal_tolerance(0.05)



    exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    exectute_trajectory_client.wait_for_server()

    rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
    group.set_named_target(arg_pose_name)
    rospy.sleep(2)
    group.go(wait = True)
    rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    

def add_scene_obj(x,y,z):
    ''' Add the scene objects, for the given transformations.
        takes the mesh from local folders.
    '''

    rospy.sleep(1)

    scene = moveit_commander.PlanningSceneInterface(synchronous = True)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "ebot_base"
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z # above the panda_hand frame
    box_name = "sphere"

    scene.add_mesh(box_name, box_pose, "/home/mustang/mustang_ws/src/eYRC-2021_Agribot/tomato_gazebo/models/tomato/meshes/tomato.obj", size=(0.7,0.7,0.7))

    rospy.loginfo('Added Object')

def get_transforms(frame, tf_buffer, rate):
    global transformations
    '''
    Function used to return transforms for the tomato.
    '''
    global x, y, z

    try:
            # transform = tf_buffer.lookup_transform('camera_depth_frame2', 'obj1','base_link', rospy.Time(0), rospy.Duration(2.0)) 
            transform = tf_buffer.lookup_transform('ebot_base', frame, rospy.Time(0), rospy.Duration(3.0)) 
            print('Done Listening. Transforms Received!') 
            transformations = True
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            transformations = False
            pass

    

def main():

    tf_buffer = tf2_ros.Buffer()
    listener =tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(5) # 5Hz

    group = moveit_commander.MoveGroupCommander('arm_planning_grp')

    nav = rospy.Publisher('navigation_feedback', Bool, queue_size= 10)
    count_pub = rospy.Publisher("perception", Int64, queue_size=10)
    rospy.sleep(1)

    rospy.Subscriber('navigation_feedback', Bool, nav_callback)
    rospy.Subscriber('direction', String, dir_callback)

    rospy.Subscriber('perception', Int64, per_callback)

    navigation_msg = std_msgs.msg.Bool()
    count_msg = std_msgs.msg.Int64()
    count_msg.data = 0

    navigation_msg.data = False
    while not rospy.is_shutdown():
        print(direction)
        if direction == "r":
            rospy.sleep(1)
            go_to_predefined('arm_planning_grp', 'view-r')
        else:
            go_to_predefined('arm_planning_grp', 'view-l')
        rate.sleep()

        if move == True:
            if count > 0:
        
                for _ in range(count):
                    if transformations != False:

                        get_transforms('obj0', tf_buffer, rate)
                        add_scene_obj(x,y,z)
                        end_eff_pose(x, y, z, 'arm_planning_grp')
                    go_to_predefined('gripper_planning_group', 'close')
                    go_to_predefined('arm_planning_grp', 'allZero')
                    go_to_predefined('gripper_planning_group', 'open')
                    if direction == "r":
                        rospy.sleep(1)
                        go_to_predefined('arm_planning_grp', 'view-r')
                    else:
                        go_to_predefined('arm_planning_grp', 'view-l')
                    rate.sleep()

            nav.publish(navigation_msg)
            rospy.sleep(1)
            count_pub.publish(count_msg)
            rospy.sleep(1)
            
        # rospy.spin()



if __name__ == '__main__':
        go_to_predefined('arm_planning_grp', 'view-l')
        
        main()

