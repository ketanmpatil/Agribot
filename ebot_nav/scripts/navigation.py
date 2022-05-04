#!/usr/bin/env python3

# Import the necessary libraries

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
import std_msgs

# Colour codes for logging
GREEN = '\033[92m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'
END = '\033[0m'
YELLOW = '\033[93m'

coordinates = {
"Turn A": {'Position': [2.461319663023336,-1.0386364336369378], 'Orientation': [0.0002949482362012977,0.0008392332900634328,0.7181915497574984,0.6958448868461601]},
"Trough_0": {'Position': [2.4544108759345964,0.06211153993850739], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_1": {'Position': [2.4544108759345964,0.7449024298373093], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_2": {'Position': [2.4544108759345964,1.5844559173542194], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_3": {'Position': [2.4544108759345964,2.3817874684787776], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_4": {'Position': [2.4544108759345964,3.161611312469203], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_5": {'Position': [2.4544108759345964,3.9807712562095072], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_6": {'Position': [2.4544108759345964,4.759607351789415], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_7": {'Position': [2.4544108759345964,5.578637737494899], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_8": {'Position': [2.4544108759345964,6.359477595666619], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_9": {'Position': [2.4544108759345964,7.1387801765796945], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},

"Turn B": {'Position': [1.948567658906576,8.545037458337159], 'Orientation': [0.0007788351250914883,0.00043177304377817774,0.9989644859679103,0.0454880507689137]},
"Turn C": {'Position': [0.9211298101523197,8.355811023325764], 'Orientation': [0.0008519850141220845,-0.00026384742636648384,0.7231887866135631,-0.6906498269184419]},
"Trough_9.1": {'Position': [0.8419087250521234,7.360900729616732], 'Orientation': [0.0008506444879436346,-0.0002647061562959105,0.7224876677290447,-0.6913832340417662]},
"Trough_8.1": {'Position': [0.8179033363489613,6.442832274036376], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
"Trough_7.1": {'Position': [0.8179033363489613,5.502765120821051], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
"Trough_6.1": {'Position': [0.8179033363489613,4.759607351789415], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
"Trough_5.1": {'Position': [0.8179033363489613,3.9807712562095072], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
"Trough_4.1": {'Position': [0.8179033363489613,3.161611312469203], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
"Trough_3.1": {'Position': [0.8179033363489613,2.3817874684787776], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
"Trough_2.1": {'Position': [0.8179033363489613,1.5844559173542194], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
"Trough_1.1": {'Position': [0.8179033363489613,0.7449024298373093], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},
"Trough_0.1": {'Position': [0.8179033363489613,0.06211153993850739], 'Orientation': [0.0008458737705708176,-0.00028198086634529995,0.7082856871307665,-0.7059253433544184]},

"Turn D": {'Position': [0.7654810553286191,-1.2619194873432018], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_10.1": {'Position': [0.7179033363489613,0.26211153993850739], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_11.1": {'Position': [0.7179033363489613,0.9449024298373093], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_12.1": {'Position': [0.7179033363489613,1.7844559173542194], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_13.1": {'Position': [0.7179033363489613,2.5817874684787776], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_14.1": {'Position': [0.7179033363489613,3.361611312469203], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_15.1": {'Position': [0.7179033363489613,4.1807712562095072], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_16.1": {'Position': [0.7179033363489613,4.959607351789415], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_17.1": {'Position': [0.7179033363489613,5.702765120821051], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_18.1": {'Position': [0.7179033363489613,6.642832274036376], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},
"Trough_19.1": {'Position': [0.719087250521234,7.360900729616732], 'Orientation': [0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425]},

"Turn E": {'Position': [0.10557982945353363,8.793129876575046], 'Orientation': [0.0007687871010552473,0.0004474898765523901,0.9979143059816102,0.06454646881468253]},
"Turn F": {'Position': [-0.7647135348541873,8.145846636604415], 'Orientation': [0.0008528336590478445,-0.0002533355833245067,0.7325148844300842,-0.6807504334072897]},

"Trough_19": {'Position': [-0.755835094290346,7.260900729616732], 'Orientation': [0, 0, -0.7068252, 0.7073883]},
"Trough_18": {'Position': [-0.755835094290346,6.442832274036376], 'Orientation': [0, 0, -0.7068252, 0.7073883]},
"Trough_17": {'Position': [-0.755835094290346,5.502765120801051], 'Orientation': [0, 0, -0.7068252, 0.7073883]},
"Trough_16": {'Position': [-0.755835094290346,4.759607351789415], 'Orientation': [0, 0, -0.7068252, 0.7073883]},
"Trough_15": {'Position': [-0.755835094290346,3.9807712562095072], 'Orientation': [0, 0, -0.7068252, 0.7073883]},
"Trough_14": {'Position': [-0.755835094290346,3.161611312469203], 'Orientation': [0, 0, -0.7068252, 0.7073883]},
"Trough_13": {'Position': [-0.755835094290346,2.3817874684787776], 'Orientation': [0, 0, -0.7068252, 0.7073883]},
"Trough_12": {'Position': [-0.755835094290346,1.5844559173542194], 'Orientation': [0, 0, -0.7068252, 0.7073883]},
"Trough_11": {'Position': [-0.755835094290346,0.7449024298373093], 'Orientation': [0, 0, -0.7068252, 0.7073883]},
"Trough_10": {'Position': [-0.755835094290346,-0.10509079094628082], 'Orientation': [0, 0, -0.7068252, 0.7073883]},

"Turn G": {'Position': [-0.7032893138045076,-1.413684556837212], 'Orientation': [0.0003015212337296583,0.000833101908383577,0.000003432569825,1]},

"End": {'Position': [0.8,-1.42], 'Orientation': [0.0003015212337296583,0.000833101908383577,0.000003432569825,1]},

}

nav_feed = None

def nav_callback(data):
    global nav_feed
    nav_feed = data.data

def nav():
    '''
    Uses the waypoint to take the Agribot to across the greenhouse.
    Also updates the manipulation node.
    '''

    rospy.init_node('navigation', anonymous=True)  # Node Initialisation

    nav = rospy.Publisher(
        'navigation_feedback', std_msgs.msg.Bool,
        queue_size=10)  # Navigation Feedback: Whether Agribot is stationary
        
    trough_pub = rospy.Publisher(
        'trough_name', std_msgs.msg.String,
        queue_size=10)  # Navgation Feedback: Position of Agribot

    nav_msg = std_msgs.msg.Bool()  # Navigation feedback msg
    trough_msg = std_msgs.msg.String()  # Navigation feedback msg

    rospy.Subscriber(
        'navigation_feedback', std_msgs.msg.Bool,
        nav_callback)  # Subscriber for returning feedback from manipulation

    ac = actionlib.SimpleActionClient(
        'move_base', MoveBaseAction)  # Starting move_base action client

    while (not ac.wait_for_server(rospy.Duration(5))):
        rospy.loginfo(
            f' {YELLOW} {BOLD} {UNDERLINE}  WAITING FOR THE SERVER TO START {END}'
        )  # Wait until the server is ready.

    rate = rospy.Rate(10)  # 10Hz

    goal = MoveBaseGoal()

    idx = 0  # Index

    goal.target_pose.header.frame_id = 'map'  # Frame Id
    goal.target_pose.header.stamp = rospy.Time.now()

    nav_msg.data = False
    nav.publish(nav_msg)

    rospy.loginfo(f' {GREEN} {BOLD} {UNDERLINE} Started Run! {END}')

    while not rospy.is_shutdown():

        try:
            # if nav_feed == False:

            # Position
            goal.target_pose.pose.position.x = coordinates[list(
                coordinates.keys())[idx]]['Position'][0]
            goal.target_pose.pose.position.y = coordinates[list(
                coordinates.keys())[idx]]['Position'][1]
            goal.target_pose.pose.position.z = 0.169

            # Orientation
            goal.target_pose.pose.orientation.x = coordinates[list(
                coordinates.keys())[idx]]['Orientation'][0]
            goal.target_pose.pose.orientation.y = coordinates[list(
                coordinates.keys())[idx]]['Orientation'][1]
            goal.target_pose.pose.orientation.z = coordinates[list(
                coordinates.keys())[idx]]['Orientation'][2]
            goal.target_pose.pose.orientation.w = coordinates[list(
                coordinates.keys())[idx]]['Orientation'][3]

            # Current Position name
            trough = list(coordinates.keys())[idx]

            trough_msg.data = str(list(coordinates.keys())[idx])
            trough_pub.publish(trough_msg)

            # Sending goal to server
            ac.send_goal(goal)
            ac.wait_for_result(rospy.Duration(60))  # Wait for 60sec

            if (ac.get_state() == GoalStatus.SUCCEEDED):
                if not "Turn" in trough.split():
                    rospy.loginfo(
                        f" {GREEN} {BOLD} {UNDERLINE}  {trough.upper()} Reached {END}"
                    )  # If bot is at a trough, Print that it has reached that trough.
                idx += 1
                nav_msg.data = True
                nav.publish(nav_msg)  # Update to manipulation node
                rospy.sleep(1)

            else:
                rospy.loginfo(
                    f" {GREEN} {BOLD} {UNDERLINE}  THE BOT DIDN'T REACH ITS DESTINATION, RETRYING {END}"
                )

            if trough == "End":  # If bot has reached the destination, print the message and signal shutdown to master.
                rospy.loginfo(
                    f" {GREEN} {BOLD} {UNDERLINE} Mission Accomplished! {END}")
                rospy.signal_shutdown()
            rate.sleep()
        except:
            pass


if __name__ == "__main__":
    nav()