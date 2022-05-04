#! /usr/bin/env python3

# Import the necessary libraries
import tf2_ros
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import std_msgs

# INITIALIZATION OF COORDINATES

x = 0
y = 0
z = 0

transformations = None

# Colour codes for logging
GREEN = '\033[92m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'
END = '\033[0m'

move = None
count = 0
trough_name = " "


def nav_callback(data):
    global move
    move = data.data


def trough_callback(data):
    global trough_name
    trough_name = data.data


def per_callback(data):
    global count
    count = data.data


rospy.init_node('manipulation_py', anonymous=True)  # ROS Node Initialization
commander = moveit_commander.roscpp_initialize(
    sys.argv)  # Moveit Commander Initialization
robot = moveit_commander.RobotCommander()  # Robot Commander


def end_eff_pose(x, y, z, planning_group):
    ''' Takes coordinates and name of the planning group as an input and commands the end to 
    that position. '''
    move_group = moveit_commander.MoveGroupCommander(planning_group)

    move_group.allow_replanning(True)
    move_group.set_planning_time(30)  # 30s
    move_group.set_goal_tolerance(0.001)  # Goal Tolerance

    # move_group.set_goal_tolerance(0.05)

    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.header.frame_id = 'ebot_base'

    # DEFAULT ORIENTATION, CALCULATED FROM MANUAL METHODS

    pose_goal.pose.orientation.y = 1

    pose_goal.pose.position.x = x
    pose_goal.pose.position.y = y - 0.27  # -0.27 IS THE OFFSET FROM ENDPOSE TO GRIPPERS
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

    group.set_goal_tolerance(0.001)  # Goal Tolerance

    exectute_trajectory_client = actionlib.SimpleActionClient(
        'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    exectute_trajectory_client.wait_for_server()

    rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) +
                  '\033[0m')

    group.set_named_target(arg_pose_name)

    rospy.sleep(2)

    group.go(wait=True)

    rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) +
                  '\033[0m')


def add_scene_obj(x, y, z):
    ''' Add the scene objects, for the given transformations.
        takes the mesh from local folders.
    '''

    rospy.sleep(1)

    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "ebot_base"
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z
    box_name = "sphere"

    scene.add_mesh(
        box_name,
        box_pose,
        "/home/mustang/mustang_ws/src/eYRC-2021_Agribot/tomato_gazebo/models/tomato/meshes/tomato.obj",
        size=(0.7, 0.7, 0.7))

    rospy.loginfo(f'{GREEN} {BOLD} {UNDERLINE} Added Object {END}')


def get_transforms(frame, tf_buffer, rate):
    global transformations
    '''
    Function used to return transforms for the tomato.
    '''
    global x, y, z

    try:
        transform = tf_buffer.lookup_transform('ebot_base', frame,
                                               rospy.Time(),
                                               rospy.Duration(3.0))
        rospy.loginfo(
            f'{GREEN} {BOLD} {UNDERLINE}  Tomato identified at {trough_name} {END}'
        )
        transformations = True
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        z = transform.transform.translation.z

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        transformations = False
        pass


def manipulator():
    '''
    1.Gets the transform of the detected tomato.
    2.Takes the end effector towards the tomato.
    3. After the tomato is placed in the basket, updates the navigation node to move.
    '''

    tf_buffer = tf2_ros.Buffer() # Transformation Buffer
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(5)  # 5Hz

    nav = rospy.Publisher('navigation_feedback', # To update navigation node
                          std_msgs.msg.Bool,
                          queue_size=10)
    count_pub = rospy.Publisher("perception",   # To set the count back to 0
                                std_msgs.msg.Int64,
                                queue_size=10)

    rospy.sleep(1)

    rospy.Subscriber('navigation_feedback', std_msgs.msg.Bool, nav_callback)
    rospy.Subscriber('trough_name', std_msgs.msg.String, trough_callback)

    rospy.Subscriber('perception', std_msgs.msg.Int64, per_callback)

    navigation_msg = std_msgs.msg.Bool()
    count_msg = std_msgs.msg.Int64()

    count_msg.data = 0

    navigation_msg.data = False

    while not rospy.is_shutdown():

        if (move == True) and (count > 0) and (trough_name.split()[0].lower()
                                               != "turn"):

            for _ in range(count):

                if transformations != False:

                    get_transforms('obj0', tf_buffer, rate) # Gets the transform of object from transform buffer.

                    add_scene_obj(x, y, z) # Adds the mesh of the tomato to planning scene.

                    end_eff_pose(x, y, z, 'arm_planning_grp') # Takes the end effector towards the tomato.a

                    go_to_predefined('gripper_planning_group', 'close') # Goes to predefined pose, close.

                    rospy.loginfo(
                        f' {GREEN} {BOLD} {UNDERLINE} Tomato picked {END}')

                    go_to_predefined('arm_planning_grp', 'allZero') # Goes to predefined pose, allZero.
                    go_to_predefined('gripper_planning_group', 'open') # Goes to predefined pose, open.

                    rospy.loginfo(
                        f" {GREEN} {BOLD} {UNDERLINE} Tomato Placed in AgriBot's Basket {END}"
                    )

                    go_to_predefined('arm_planning_grp', 'view') # Goes to predefined pose, open.
                    rate.sleep()

        nav.publish(navigation_msg)
        rospy.sleep(1)

        count_pub.publish(count_msg)
        rospy.sleep(1)


if __name__ == '__main__':
    go_to_predefined('arm_planning_grp', 'view') # Goes to predefined pose, open.

    manipulator()