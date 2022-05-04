#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <list>
#include <vector>
#include <bits/stdc++.h>

using std::string;
using std::cout;
using std::cin;
using std::map;
using std::list;
using std::vector;

bool nav_feed = false;
int i = 0;

float turns[8][6] =
 {
    // {2.461319663023336,-1.0386364336369378,0.0002949482362012977,0.0008392332900634328,0.7181915497574984,0.6958448868461601},
    // {1.948567658906576,8.545037458337159,0.0007788351250914883,0.00043177304377817774,0.9989644859679103,0.0454880507689137},
    // {0.9211298101523197,8.355811023325764,0.0008519850141220845,-0.00026384742636648384,0.7231887866135631,-0.6906498269184419},
    // {0.7654810553286191,-1.2619194873432018,0.00028343464247164234,0.0008418222665757848,0.7082335906750342,0.7059776144040425},
    {0.10557982945353363,8.793129876575046,0.0007687871010552473,0.0004474898765523901,0.9979143059816102,0.06454646881468253},
    {-0.7647135348541873,8.145846636604415,0.0008528336590478445,-0.0002533355833245067,0.7325148844300842,-0.6807504334072897},
    {-0.7032893138045076,-1.413684556837212,0.0003015212337296583,0.000833101908383577,0.000003432569825,1},
    {0.8,-1.42,0.0003015212337296583,0.000833101908383577,0.000003432569825,1},
};


class Navigation{
    private:
        ros::NodeHandle node;
        ros::Publisher nav_;
        ros::Publisher trough_pub_ ;
        ros::Subscriber nav_sub_;
        ros::Subscriber aruco_sub_;

    public:
        Navigation(){
        nav_ = node.advertise<std_msgs::Bool>("navigation_feedback", 1000, true);
        trough_pub_ = node.advertise<std_msgs::String>("trough_name",1000, true);
        nav_sub_ = node.subscribe("navigation_feedback",1000 ,&Navigation::navCallback, this);
        aruco_sub_ = node.subscribe("aruco_pub",1 ,&Navigation::arucoCallback, this);
        }

    void navCallback(const std_msgs::Bool::ConstPtr& msg){
        nav_feed = msg->data;

    }

    void arucoCallback(const std_msgs::Int16::ConstPtr& msg){

        std_msgs::Bool nav_msg;
        std_msgs::String trough_msg;
        int idx = 0;
        ROS_INFO("inside");
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base");
    
        while (! ac.waitForServer(ros::Duration(5))){
            ROS_INFO("Waiting for Server");
        };

        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        nav_msg.data = false;
        nav_.publish(nav_msg);

        try{
            if (nav_feed == false){
                
                goal.target_pose.pose.position.x = turns[idx][0];
                goal.target_pose.pose.position.y = turns[idx][1];
                goal.target_pose.pose.position.z = 0.619;

                goal.target_pose.pose.orientation.x = turns[idx][2];
                goal.target_pose.pose.orientation.y = turns[idx][3];
                goal.target_pose.pose.orientation.z = turns[idx][4];
                goal.target_pose.pose.orientation.w = turns[idx][5];

                ros::Duration(0.1).sleep();

                ac.sendGoal(goal);

                ros::Duration(0.5).sleep();

                // cout << i << std::endl;
                // cout << aruco -1 << std::endl;

                if (i == msg->data - 1){
                    ac.cancelGoal();
                    nav_msg.data = true;
                    trough_msg.data = "Trough " + std::to_string(i);
                    ROS_INFO("Trough %s", trough_msg.data);
                    trough_pub_.publish(trough_msg);
                    nav_.publish(nav_msg);
                    i = msg->data;
                }
                
                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    idx += 1;
                    trough_msg.data = "Turn";
                    trough_pub_.publish(trough_msg);
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


    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "navigation");

    Navigation navigation;
    ros::spin();
    return 0;
}