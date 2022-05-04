#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <cmath>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;
cv::Mat cv_ptr;
int id = -1;

vector<int> getPoint(vector<vector<cv::Point2f>> &corners)
{
    vector<cv::Point2f> a = corners.at(0);
    cv::Point2f p1 = a.at(0);
    cv::Point2f p2 = a.at(2);

    int X = p1.x;
    int Y = p2.x;

    vector<int> coordinate = {X, Y};

    return coordinate;
}

class ArucoDetection
{

private:
    ros::NodeHandle node;
    ros::Publisher aruco_pub_;
    ros::Subscriber aruco_img_;
    // ros::Subscriber aruco_sub_;

public:
    ArucoDetection()
    {
        aruco_pub_ = node.advertise<std_msgs::Int16>("aruco_pub", 1000, true);
        // aruco_sub_ = node.subscribe("aruco_pub", 1000, callback);
        aruco_img_ = node.subscribe("/ebot/camera1/image_raw", 1000, &ArucoDetection::imgCallback, this);
    }

    void imgCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        std_msgs::Header msg_header = msg->header;
        std_msgs::Int16 aruco_msg;

        std::string frame_id = msg_header.frame_id.c_str();
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;

        cv_bridge::CvImagePtr cv_ptr;

        cv::Rect crop_region(100, 100, 700, 700);

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
           
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img = cv_ptr->image;
        img = img(crop_region);
        // cv::drawMarker(img, cv::Point(img.cols / 2, img.rows / 2), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
        
        try
        {
            cv::aruco::detectMarkers(img, dictionary, corners, ids);
            

            if (ids.size() > 0)
            {
                // cv::aruco::drawDetectedMarkers(img, corners, ids);
                vector<int> coordinate = getPoint(corners);
                
                if (id != ids.at(0))
                {
                    aruco_msg.data = ids.at(0);
                    aruco_pub_.publish(aruco_msg);
                    id = ids.at(0);
                }
            };

            // cv::imshow("out", img);
            // cv::waitKey(3);
        }
        catch (cv::Exception &e)
        {
            ROS_INFO("Error: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco");
    
    ArucoDetection arucoDect;
    // ros::Rate rate(0);
    ros::spin();
    // rate.sleep();
    return 0;
}