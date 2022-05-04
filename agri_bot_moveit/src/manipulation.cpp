#include "ros/ros.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <iostream>
#include <boost/lexical_cast.hpp>

using std::string;
using std::cout;
using std::endl;


float x = 0;
float y = 0;
float z = 0;
int count = 0;

bool transformations = false;
bool move = false;

string trough_name = " ";


void endEffPose(float x, float y, float z, string planning_group){

    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    
    move_group.allowReplanning(true);
    move_group.setPlanningTime(30);
    move_group.setGoalTolerance(0.001);

    geometry_msgs::PoseStamped pose_goal;
    pose_goal.header.frame_id = "ebot_base";

    pose_goal.pose.orientation.y = 1;

    pose_goal.pose.position.x = x;
    pose_goal.pose.position.y = y - 0.27;  // -0.27 IS THE OFFSET FROM ENDPOSE TO GRIPPERS
    pose_goal.pose.position.z = z;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    move_group.setPoseTarget(pose_goal);
    move_group.move();
        // move_group.stop();
        // move_group.clearPoseTargets();

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

};

void gotoPredefined( string planning_group, string arg_pose_name){

    moveit::planning_interface::MoveGroupInterface move_group_pre(planning_group);

    move_group_pre.allowReplanning(true);
    move_group_pre.setPlanningTime(30);
    move_group_pre.setGoalTolerance(0.001);
    move_group_pre.allowLooking(true);

    actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> exectute_trajectory_client("execute_trajectory");
    exectute_trajectory_client.waitForServer();

    // ROS_INFO_NAMED("Client", "Going to pose %s", arg_pose_name);

    move_group_pre.setNamedTarget(arg_pose_name);

    ros::Duration(1.0).sleep();

    move_group_pre.move();

    // move_group_pre.stop

    // ROS_INFO_NAMED("Client", "Now at pose %s", arg_pose_name);

}

void addSceneObj(float x, float y, float z, moveit::planning_interface::PlanningSceneInterface &scene, moveit_msgs::CollisionObject &collision_object){
    
    collision_object.header.frame_id = "ebot_base";

    collision_object.id = "Tomato";

    geometry_msgs::Pose collision_pose;
    collision_pose.position.x = x;
    collision_pose.position.y = y;
    collision_pose.position.z = z;
    collision_pose.orientation.w = 1;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(1);
    primitive.dimensions[0] = 0.035;


    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(collision_pose);
    collision_object.operation = collision_object.ADD;
    scene.applyCollisionObject(collision_object);

    ROS_INFO_NAMED("Collision", "Added Object");

};

void getTransforms(string frame){
    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::Buffer tfbuffer;
    tf2_ros::TransformListener listner(tfbuffer);
    try{
        transformStamped = tfbuffer.lookupTransform("ebot_base", frame,
                               ros::Time(0), ros::Duration(3.0));
        transformations = true;
        x = transformStamped.transform.translation.x;
        y = transformStamped.transform.translation.y;
        z = transformStamped.transform.translation.z;
        ROS_INFO("TRANSFOR RECIEVED");
    }
    catch (tf2::TransformException &ex) {
        ROS_INFO("TRANSFOR NOT RECIEVED");
      transformations = false;
      
    }

};

void navCallback(const std_msgs::Bool::ConstPtr& msg){
    move = msg->data;
};

void nameCallback(const std_msgs::String::ConstPtr& msg){
    trough_name = msg->data;
};

void imgCallback(const std_msgs::Int16::ConstPtr& msg){
    count = msg->data;
};

int main(int argc, char** argv){

    ros::init(argc, argv, "manipulation" );
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    ros::NodeHandle node;

    ros::Rate rate(10);

    ros::Publisher nav = node.advertise<std_msgs::Bool>("navigation_feedback", 1000);
    ros::Publisher count_pub = node.advertise<std_msgs::Int16>("perception", 1000, true);

    ros::Subscriber sub = node.subscribe("navigation_feedback", 1000, navCallback);
    ros::Subscriber name_sub = node.subscribe("trough_name", 1000, nameCallback);
    ros::Subscriber img_sub = node.subscribe("perception", 1000, imgCallback);

    moveit::planning_interface::PlanningSceneInterface scene;

    moveit_msgs::CollisionObject collision_object;

    std_msgs::Bool navigation_msg;
    std_msgs::Int16 count_msg;

    navigation_msg.data = false;
    count_msg.data = 0;
    int l = 0;
    // gotoPredefined("arm_planning_grp", "view");
    int alt = 0;
    while (node.ok()){
        ros::spinOnce();
        if(alt == 0){
            gotoPredefined("arm_planning_grp", "view");
            alt = 1;
        }   
        // for(int j = 0; j <= count; j++){            
        //     getTransforms("obj");
        //     addSceneObj(x, y, z, scene, collision_object);
        //     endEffPose(x, y, z, "arm_planning_grp");
        //     gotoPredefined("gripper_planning_group", "close");
        //     gotoPredefined("arm_planning_grp", "allZero");
        //     gotoPredefined("gripper_planning_group", "open");
        //     gotoPredefined("arm_planning_grp", "view");
        if (move == 1){
            if( count > 0){
                for (int i = 0; i <= count; i++){
                    if (transformations != false);
                        
                        getTransforms("obj0");

                        addSceneObj(x, y, z, scene, collision_object);

                        endEffPose(x, y, z, "arm_planning_grp");

                        gotoPredefined("gripper_planning_group", "close");

                        // ROS_INFO_NAMED("Action", "Tomato Picked");

                        gotoPredefined("arm_planning_grp", "allZero");
                        gotoPredefined("gripper_planning_group", "open");

                        // ROS_INFO_NAMED("Action", "Tomato Placed in AgriBot's Basket");

                        gotoPredefined("arm_planning_grp", "view");
                
                };
            };
            nav.publish(navigation_msg);
            count_pub.publish(count_msg);
    
    };
    rate.sleep();
}
    return 0;
};

