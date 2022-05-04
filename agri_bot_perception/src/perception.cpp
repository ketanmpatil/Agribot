#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <std_msgs/String.h>



float cy = 240.5;
float cx = 320.5;
float f = 554.387;

float X;
float Y;
float Z;



class Perception{
    private:
        ros::NodeHandle node;
        ros::Subscriber image;
        ros::Subscriber depth_image;
        ros::Publisher count_feed;
        tf2_ros::TransformBroadcaster broadcaster;
        cv::InputArray redLower = (0,12,12);
        cv::InputArray redUpper = (0,213,225);
        cv::Mat dpt_img;
        

    public:
        Perception(){
            image = node.subscribe<sensor_msgs::Image>("/camera/color/image_raw2",1000,&Perception::imgCallback, this);
            depth_image = node.subscribe<sensor_msgs::Image>("/camera/depth/image_raw2",1000,&Perception::depthCallback, this);
            count_feed = node.advertise<std_msgs::Int16>("perception",1000,true);
        }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr dpt;
        dpt = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        dpt_img = dpt->image;
    }

    void imgCallback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr img;
        cv::Mat hsv_img;
        cv::Mat mask;
        std_msgs::Int16 count_tom; 
        // cv::OutputArray mask;
        
        img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(img->image, hsv_img, cv::COLOR_BGR2HSV);
        std::vector<std::vector<cv::Point>> contours;
        cv::inRange(hsv_img, cv::Scalar(0,12,12), cv::Scalar(0,213,225), mask);

        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);

        // cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 6);

        cv::findContours(mask, contours,cv::RETR_TREE,cv::CHAIN_APPROX_NONE);
        int count = 0;

        for(int i = 0; i < contours.size(); i++){
            
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(cv::InputArray(contours.at(i)), center, radius);
            
            std::string text = "Tomato";
            float fontScale = 0.5;
            cv::Scalar color = (255, 0, 0);
            int thickness = 1;
            float depth = dpt_img.at<float>(center.y, center.x);

            // std::cout << center.x << std::endl;
            // std::cout << center.y << std::endl;
            // std::cout << depth << std::endl;

            X = depth * (center.x - cx) / f;
            Y = depth * (center.y - cy) / f;
            Z = depth;
            std_msgs::String name;
            

            if ( Z < 0.9){
                name.data = "obj" + std::to_string(count);
                geometry_msgs::TransformStamped t;
                t.header.stamp = ros::Time::now();
                t.header.frame_id = "camera_depth_frame2";
                t.child_frame_id = name.data  ;

                t.transform.translation.x = X;
                t.transform.translation.y = Y;
                t.transform.translation.z = Z;

                t.transform.rotation.x = 0;
                t.transform.rotation.y = 0;
                t.transform.rotation.z = 0;
                t.transform.rotation.w = 1;

                broadcaster.sendTransform(t);
                count += 1;

            }
            cv::putText(img->image, text , center, cv::FONT_HERSHEY_SIMPLEX,fontScale, color, thickness, cv::LINE_AA);

        }
        cv::drawContours(img->image, contours, -1, (0,255,0), 3);
        cv::imshow("RGM Image", img->image);
        cv::waitKey(1);

        try{
            if (Z < 0.9){
                count_tom.data = contours.size();
                count_feed.publish(count_tom);
            }
        }
        catch(...){
            count_tom.data = 0;
            count_feed.publish(count_tom);          
        }
    }


};

int main(int argc, char** argv){

    ros::init(argc, argv, "Perception");

    Perception perception;

    ros::Rate rate(10);

    rate.sleep();
    ros::spin();
    

    return 0;
}

