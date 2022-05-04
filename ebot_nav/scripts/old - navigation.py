#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
import tf2_ros
import math
import std_msgs
# from ebot_gazebo.msg import FeedbackAction
x = 0
y = 0
z = 0

coordinates = { 
# 'Turn1': {'Position': [2.471319663023336,-1.1486364336369378], 'Orientation': [0.0002949482362012977,0.0008392332900634328,0.7181915497574984,0.6958448868461601]},
# 'T1': {'Position': [2.4544108759345964,0.06211153993850739], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
# 'T2': {'Position': [2.4544108759345964,0.7449024298373093], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
# 'T3': {'Position': [2.4544108759345964,1.5844559173542194], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
# 'T4': {'Position': [2.4544108759345964,2.3817874684787776], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
# 'T5': {'Position': [2.4544108759345964,3.161611312469203], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
# 'T6': {'Position': [2.4544108759345964,3.9807712562095072], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
# 'T7': {'Position': [2.4544108759345964,4.759607351789415], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
# 'T8': {'Position': [2.4544108759345964,5.578637737494899], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
# 'T9': {'Position': [2.4544108759345964,6.359477595666619], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
# 'T10': {'Position': [2.4544108759345964,7.1387801765796945], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},

# 'Turn2': {'Position': [2.348567658906576,8.545037458337159], 'Orientation': [0.0007788351250914883,0.00043177304377817774,0.9989644859679103,0.0454880507689137]},
# 'Turn2.1': {'Position': [0.9211298101523197,8.595811023325764], 'Orientation': [0.0008519850141220845,-0.00026384742636648384,0.7231887866135631,-0.6906498269184419]},
# 'T10.1': {'Position': [0.8419087250521234,7.360900729616732], 'Orientation': [0.0008506444879436346,-0.0002647061562959105,0.7224876677290447,-0.6913832340417662]},
# 'T9.1': {'Position': [0.8179033363489613,6.442832274036376], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T8.1': {'Position': [0.8179033363489613,5.502765120821051], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T7.1': {'Position': [0.8179033363489613,4.759607351789415], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T6.1': {'Position': [0.8179033363489613,3.9807712562095072], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T5.1': {'Position': [0.8179033363489613,3.161611312469203], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T4.1': {'Position': [0.8179033363489613,2.3817874684787776], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T3.1': {'Position': [0.8179033363489613,1.5844559173542194], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T2.1': {'Position': [0.8179033363489613,0.7449024298373093], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T1.1': {'Position': [0.8179033363489613,0.06211153993850739], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},

'Turn3': {'Position': [0.7854810553286191,-1.2619194873432018], 'Orientation': [0.0007788351250914883,0.00043177304377817774,-0.9989644859679103,0.0454880507689137]},
'Turn3.1': {'Position': [-0.9932893138045076,-1.403684556837212], 'Orientation': [0.0003015212337296583,0.000833101908383577,0.7192883432569825,0.6947110869136761]},
# 'T11': {'Position': [-0.7268835094290346,-0.10509079094628082], 'Orientation': [0.00027052226142558293,0.0008542476213070786,0.6931745608444574,0.7207691900163335]},
# 'T12': {'Position': [-0.7268835094290346,0.7449024298373093], 'Orientation': [0.00027052226142558293,0.0008542476213070786,0.6931745608444574,0.7207691900163335]},
# 'T13': {'Position': [-0.7268835094290346,1.5844559173542194], 'Orientation': [0.00027052226142558293,0.0008542476213070786,0.6931745608444574,0.7207691900163335]},
# 'T14': {'Position': [-0.7268835094290346,2.3817874684787776], 'Orientation': [0.00027052226142558293,0.0008542476213070786,0.6931745608444574,0.7207691900163335]},
# 'T15': {'Position': [-0.7268835094290346,3.161611312469203], 'Orientation': [0.00027052226142558293,0.0008542476213070786,0.6931745608444574,0.7207691900163335]},
# 'T16': {'Position': [-0.7268835094290346,3.9807712562095072], 'Orientation': [0.00027052226142558293,0.0008542476213070786,0.6931745608444574,0.7207691900163335]},
# 'T17': {'Position': [-0.7268835094290346,4.759607351789415], 'Orientation': [0.00027052226142558293,0.0008542476213070786,0.6931745608444574,0.7207691900163335]},
# 'T18': {'Position': [-0.7268835094290346,5.502765120821051], 'Orientation': [0.00027052226142558293,0.0008542476213070786,0.6931745608444574,0.7207691900163335]},
# 'T19': {'Position': [-0.7268835094290346,6.442832274036376], 'Orientation': [0.00027052226142558293,0.0008542476213070786,0.6931745608444574,0.7207691900163335]},
'T20': {'Position': [-0.7268835094290346,7.360900729616732], 'Orientation': [0.00027052226142558293,0.0008542476213070786,0.6931745608444574,0.7207691900163335]},

'Turn4': {'Position': [-0.7268835094290346,8.545037458337159], 'Orientation': [0.0007788351250914883,0.00043177304377817774,0,1]},
'Turn4.1': {'Position': [0.9211298101523197,8.595811023325764], 'Orientation': [0.0008519850141220845,-0.00026384742636648384,0.7231887866135631,-0.6906498269184419]},
# 'T20.1': {'Position': [0.8419087250521234,7.360900729616732], 'Orientation': [0.0008506444879436346,-0.0002647061562959105,0.7224876677290447,-0.6913832340417662]},
# 'T19.1': {'Position': [0.8179033363489613,6.442832274036376], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T18.1': {'Position': [0.8179033363489613,5.502765120821051], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T17.1': {'Position': [0.8179033363489613,4.759607351789415], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T16.1': {'Position': [0.8179033363489613,3.9807712562095072], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T15.1': {'Position': [0.8179033363489613,3.161611312469203], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T14.1': {'Position': [0.8179033363489613,2.3817874684787776], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T13.1': {'Position': [0.8179033363489613,1.5844559173542194], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T12.1': {'Position': [0.8179033363489613,0.7449024298373093], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
# 'T11.1': {'Position': [0.8179033363489613,0.06211153993850739], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},

'Origin': {'Position': [0.8,-1.42], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},


}


nav_call = None

def nav_callback(data):
    global nav_call
    nav_call = data.data

def get_transforms(tf_buffer):
    global transformations
    '''
    Function used to return transforms for the tomato.
    '''
    global x, y, z

    try:
            # transform = tf_buffer.lookup_transform('camera_depth_frame2', 'obj1','base_link', rospy.Time(0), rospy.Duration(2.0)) 
            transform = tf_buffer.lookup_transform('ebot_base', 'aruco', rospy.Time(0), rospy.Duration(3.0)) 
            print('Done Listening. Transforms Received!') 
            transformations = True
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            transformations = False
            pass

    return [x,y,z]

def nav():

    rospy.init_node('nav', anonymous= True)

    nav = rospy.Publisher('navigation_feedback', std_msgs.msg.Bool, queue_size=10)
    dir = rospy.Publisher('direction', std_msgs.msg.String, queue_size=10)
    vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    nav_msg = std_msgs.msg.Bool()
    dir_msg = std_msgs.msg.String()
    vel_msg = Twist()
    rospy.Subscriber('navigation_feedback', Bool, nav_callback)

    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # while(not ac.wait_for_server(rospy.Duration(5))):
    #     rospy.loginfo('WAITING FOR THE SERVER TO START')

    rate = rospy.Rate(10)
    goal = MoveBaseGoal()

    idx = 0

    goal.target_pose.header.frame_id ='map'
    goal.target_pose.header.stamp = rospy.Time.now()
    nav_msg.data = False
    nav.publish(nav_msg)
        
    tf_buffer = tf2_ros.Buffer()
    listener =tf2_ros.TransformListener(tf_buffer)
    while not rospy.is_shutdown():
        # dir.publish(dir_msg)
        try:
            vel_msg.linear.x = 0.3
            vel.publish(vel_msg)
            cords = get_transforms(tf_buffer)
            print(cords)
            
            # print(dist)
            if x < 0.03 and x > -0.03:
                vel_msg.linear.x=0
                vel.publish(vel_msg)
                break
            
        except:
            raise
        
        # try:
        #     if nav_call == False:
        #         # coordinate = coordinates[idx]

        #         # goal.target_pose.pose.position=Point(coordinate.position.x,coordinate.position.y,0.169)#contains a point in free space
        #         # goal.target_pose.pose.orientation = coordinate.orientation
        #         goal.target_pose.pose.position.x=coordinates[list(coordinates.keys())[idx]]['Position'][0]
        #         goal.target_pose.pose.position.y=coordinates[list(coordinates.keys())[idx]]['Position'][1]
        #         goal.target_pose.pose.position.z=0.169
        #         goal.target_pose.pose.orientation.x = coordinates[list(coordinates.keys())[idx]]['Orientation'][0]
        #         goal.target_pose.pose.orientation.y = coordinates[list(coordinates.keys())[idx]]['Orientation'][1]
        #         goal.target_pose.pose.orientation.z = coordinates[list(coordinates.keys())[idx]]['Orientation'][2]
        #         goal.target_pose.pose.orientation.w = coordinates[list(coordinates.keys())[idx]]['Orientation'][3]

        #         rospy.loginfo(f"SENDING GOAL INFORMATION {list(coordinates.keys())[idx]}")
        #         if list(coordinates.keys())[idx] == "Turn3":
        #             dir_msg.data = "r"
        #         ac.send_goal(goal)
        #         ac.wait_for_result(rospy.Duration(60))
                

        #         if (ac.get_state() == GoalStatus.SUCCEEDED):
        #             rospy.loginfo("REACHED THE DESTINATION")
        #             idx += 1
        #             # pub.publish(pub_msg)
        #             nav_msg.data = True
        #             nav.publish(nav_msg)
        #             rospy.sleep(1) 
        #         else:
        #             rospy.loginfo("THE BOT DIDN'T REACH ITS DESTINATION, RETRYING")
        #         rate.sleep()
        # except:
        #     pass
        # # rospy.spin()
if __name__ == "__main__":
    nav()