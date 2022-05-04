#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16.h>


void arucoCallback( const std_msgs::Int16::ConstPtr& msg ){
    std::cout << msg->data << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv,"aruco_trial");

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("aruco_pub",1000,arucoCallback);
    ros::spin();
    return 0;
}