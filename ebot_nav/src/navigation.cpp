#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include <geometry_msgs/Twist.h>
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <list>
#include <vector>
#include <bits/stdc++.h>
#include <boost/lexical_cast.hpp>

using std::string;
using std::cout;
using std::cin;
using std::map;
using std::list;
using std::vector;

bool nav_feed = false;
int aruco = -1;
int i = -1;

float turns[8][6] =
 {
    // {2.461319663023336,-1.0386364336369378,0.0002949482362012977,0.0008392332900634328,0.7181915497574984,0.6958448868461601},
    // {1.948567658906576,8.545037458337159,0.0007788351250914883,0.00043177304377817774,0.9989644859679103,0.0454880507689137},
    // {0.9211298101523197,8.355811023325764,0.0008519850141220845,-0.00026384742636648384,0.7231887866135631,-0.6906498269184419},
    // {0.7654810553286191,-1.2619194873432018,0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425},
    {0.7812139359303312,8.331166285041608,0.000765106010255093,0.0004474898765523901,0.9996064093700745,0.028041243440536986},
    {-0.7453050961468395,8.472045110029118,0.0008528336590478445,-0.0002533355833245067,0.7351898638321094,-0.6778606552971833},
    {-0.9772632960755324,-1.024441534230481,0.0003015212337296583,0.000833101908383577,0.000003432569825,1},
    {0.8,-1.42,0.0003015212337296583,0.000833101908383577,0.000003432569825,1},
};

void navCallback(const std_msgs::Bool::ConstPtr& msg){
    nav_feed = msg->data;
};

void arucoCallback(const std_msgs::Int16::ConstPtr& msg){
    aruco = msg->data;
};


int main(int argc, char** argv){

    ros::init(argc, argv, "navigation");

    ros::NodeHandle node;

    ros::Publisher nav = node.advertise<std_msgs::Bool>("navigation_feedback", 1000, true);
    ros::Publisher trough_pub = node.advertise<std_msgs::String>("trough_name",1000, true);
    ros::Publisher cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel",1000, true);

    std_msgs::Bool nav_msg;
    std_msgs::String trough_msg;

    ros::Subscriber nav_sub = node.subscribe("navigation_feedback",1000 ,navCallback);
    ros::Subscriber aruco_sub = node.subscribe("aruco_pub",1000 ,arucoCallback);
    

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base");
    
    while (! ac.waitForServer(ros::Duration(5))){
        ROS_INFO("Waiting for Server");
    };

    ros::Rate rate(10);

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    nav_msg.data = false;
    nav.publish(nav_msg);

    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.angular.x = 0;
    ROS_INFO("Started RUN!");

    int idx = 0;
    while(node.ok()){
        ros::spinOnce();
        try{
            if (nav_feed == false){
                // if(true){
                
                goal.target_pose.pose.position.x = turns[idx][0];
                goal.target_pose.pose.position.y = turns[idx][1];
                goal.target_pose.pose.position.z = 0.619;

                goal.target_pose.pose.orientation.x = turns[idx][2];
                goal.target_pose.pose.orientation.y = turns[idx][3];
                goal.target_pose.pose.orientation.z = turns[idx][4];
                goal.target_pose.pose.orientation.w = turns[idx][5];

                // ros::Duration(0.1).sleep();
                ac.sendGoal(goal);

                if (i == aruco - 1){
                    ac.cancelGoal();
                    cmd_vel.publish(vel);
                    nav_msg.data = true;
                    trough_msg.data = "Trough " + boost::lexical_cast<string>(aruco);;
                    // ROS_INFO("Trough %s", trough_msg.data);
                    std::cout << trough_msg.data << std::endl;
                    trough_pub.publish(trough_msg);
                    nav.publish(nav_msg);
                    i = aruco;
                }
                
                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    idx += 1;
                    trough_msg.data = "Turn";
                    trough_pub.publish(trough_msg);
                }

                if (idx == 7){
                    ROS_INFO("Mission Accomplished");
                    ros::shutdown();
                }

            }

        }

        catch(...){
            ROS_INFO("Hi");
        }
        rate.sleep();
    };



}

// pose: 
//   pose: 
//     position: 
//       x: 0.7812139359303312
//       y: 8.331166285041608
//       z: 0.0
//     orientation: 
//       x: 0.000765106010255093
//       y: 0.00036003523280611063
//       z: 0.9996064093700745
//       w: 0.028041243440536986

// pose: 
//     position: 
//       x: -0.7453050961468395
//       y: 8.472045110029118
//       z: 0.0
//     orientation: 
//       x: 0.0008566378314606872
//       y: -0.00024958009440506266
//       z: 0.7351898638321094
//       w: -0.6778606552971833

// pose: 
//   pose: 
//     position: 
//       x: -0.9772632960755324
//       y: -1.024441534230481
//       z: 0.0
//     orientation: 
//       x: 0.00041788971694721195
//       y: -0.0007861694283819592
//       z: 0.025021420769396292
//       w: -0.9996865187690065

