#! /usr/bin/env python3
from importlib.resources import path
import tf2_ros
import sys
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import math
import geometry_msgs.msg
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool, Int64
from moveit_msgs.msg import Grasp, CollisionObject, ExecuteTrajectoryAction
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped

# INITIALIZATION OF COORDINATES 
x = 0
y = 0
z = 0

move = False
count = 0

def moveit_callback(data):
    global move
    move = data

def perception_callback(arg):
    global count
    count = arg

rospy.init_node('manipulation', anonymous=True) # ROS Node Initialization
commander = moveit_commander.roscpp_initialize(sys.argv) # Moveit Commander Initialization
robot = moveit_commander.RobotCommander() # Robot Commander
group = moveit_commander.MoveGroupCommander('arm_planning_grp')
scene = moveit_commander.PlanningSceneInterface()
collision_obj = CollisionObject()

def addCollisionObject(scene,x,y,z):
    

    collision_obj.header.frame_id = 'ebot_base'
    collision_obj.id = 'sphere'
    collision_obj.pose.orientation.w=1

    obj = SolidPrimitive()
    obj.type = SolidPrimitive.SPHERE
    obj.dimensions = [0.02]


    collision_obj.primitives.append(obj)
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.y = 1
    collision_obj.primitive_poses.append(pose.pose)
    collision_obj.operation = collision_obj.ADD
    # print(collision_obj)
    rospy.sleep(1)

    scene.add_object(collision_obj)

def constraints(pose, group):
    constraint = moveit_msgs.msg.Constraints()
    constraint.name = 'ebot_constraints'

    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    path_constraint = moveit_msgs.msg.PositionConstraint()

    orientation_constraint.header = pose.header
    orientation_constraint.link_name = group.get_end_effector_link()
    orientation_constraint.orientation = pose.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = math.radians(100) #Z 100
    orientation_constraint.absolute_y_axis_tolerance = math.radians(300) # 260
    orientation_constraint.absolute_z_axis_tolerance = math.radians(360)
    orientation_constraint.weight = 1

    constraint.orientation_constraints.append(orientation_constraint)

    return constraint
    



def end_eff_pose(x, y, z, planning_group):
    ''' Takes coordinates and name of the planning group as an input and commands the end to 
    that position. '''

    group.allow_replanning(True)
    group.set_planning_time(30.0)
    # group.set_max_acceleration_scaling_factor(0.5)
    # group.set_max_velocity_scaling_factor(0.1)

    group.set_goal_tolerance(0.0001)
    # constraint = constraints(group.get_current_pose(), group)
    # group.set_path_constraints(constraint)
    # group
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
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()
        group.execute(plan)
    except:
        pass

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



    exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    exectute_trajectory_client.wait_for_server()

    rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
    group.set_named_target(arg_pose_name)
    group.go(wait = True)
    rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    

def add_scene_obj(x,y,z):
    ''' Add the scene objects, for the given transformations.
        takes the mesh from local folders.
    '''


    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "ebot_base"
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z # above the panda_hand frame
    box_name = "sphere"

    scene.add_mesh(box_name, box_pose, "/home/wolf/wolf_ws/src/eYRC-2021_Agribot-master/tomato_gazebo/models/tomato/meshes/tomato.obj", size=(0.7,0.7,0.7))

    rospy.loginfo('Added Object')

def get_transforms(frame, tf_buffer, rate):
    '''
    Function used to return transforms for the tomato.
    '''
    global x, y, z

    try:
            # transform = tf_buffer.lookup_transform('camera_depth_frame2', 'obj1','base_link', rospy.Time(0), rospy.Duration(2.0)) 
            transform = tf_buffer.lookup_transform('ebot_base', frame, rospy.Time(0), rospy.Duration(3.0)) 
            print('Done Listening. Transforms Received!')
            
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            get_transforms(frame, tf_buffer, rate)

    x = transform.transform.translation.x
    y = transform.transform.translation.y
    z = transform.transform.translation.z

def main():

    tf_buffer = tf2_ros.Buffer()
    listener =tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(5) # 5Hz

    
    moveit_pub = rospy.Publisher('moveit_feedback', Bool, queue_size= 10)
    perception_pub = rospy.Publisher('perception_feedback', Int64, queue_size=10)

    rospy.Subscriber('moveit_feedback', Bool, moveit_callback)
    # rospy.Subscriber('perception_feedback', Int64, perception_callback)
    rospy.Subscriber('p', Int64, perception_callback)
    i = 0
    while not rospy.is_shutdown():

        # if move == True:
        #     if count != None or count > 0:
        #         for _ in range(count):
        #             get_transforms('obj0', tf_buffer, rate)
        #             add_scene_obj(x,y,z)
        #             end_eff_pose(x, y, z, 'arm_planning_grp')
        #             go_to_predefined('gripper_planning_group', 'close')
        #             go_to_predefined('arm_planning_grp', 'allZero')
        #             go_to_predefined('gripper_planning_group', 'open')

        #             rate.sleep()
        #         moveit_pub.publish(False)
        #         perception_pub.publish(0)

        go_to_predefined('arm_planning_grp', 'view-l')
        # for _ in range(count.data):
        #     obj = "obj" + str(i)
        #     get_transforms(obj, tf_buffer, rate)
        #     addCollisionObject(scene, x, y, z)
        #     print(f"added object {obj}")
        #     i += 1
        #     rospy.sleep(1)
        # constraints(group.get_current_pose(), group)
        get_transforms('obj0', tf_buffer, rate)
        add_scene_obj(x,y,z)
        end_eff_pose(x, y, z, 'arm_planning_grp')
        go_to_predefined('gripper_planning_group', 'close')
        go_to_predefined('arm_planning_grp', 'allZero')
        go_to_predefined('gripper_planning_group', 'open')

        rate.sleep()


    moveit_commander.roscpp_shutdown()

    

if __name__ == "__main__":
    main()

