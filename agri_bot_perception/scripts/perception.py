#!/usr/bin/env python3

# Import the necessary libraries

import rospy  # Python library for ROS
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
import tf2_ros
import tf_conversions
import geometry_msgs
from std_msgs.msg import Int16

redLower = (0, 12, 12)
redUpper = (0, 213, 225)

cy = 240.5  # cy and cx are coordinates for center of camers
cx = 320.5
f = 554.387  # Focal length

img = None
dpt = None

# Colour codes for logging
GREEN = '\033[92m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'
END = '\033[0m'

br = CvBridge()

broadcaster = tf2_ros.TransformBroadcaster()


def img_callback(msg):  # RGB Image callback function
    global img
    img = br.imgmsg_to_cv2(msg, 'bgr8')
    img = np.array(img)


def depth_callback(dpt_msg):  # Depth Image callback function
    global dpt
    dpt = br.imgmsg_to_cv2(dpt_msg, '32FC1')
    dpt = np.array(dpt, dtype=np.dtype('float32'))


def main():

    count_feed = rospy.Publisher(
        'perception', Int16,
        queue_size=10)  # Publisher node to publish count of tomatoes

    rospy.Subscriber("/camera/color/image_raw2", Image,
                     img_callback)  # Subscribe to camera topics
    rospy.Subscriber("/camera/depth/image_raw2", Image, depth_callback)

    rate = rospy.Rate(10)  # 10Hz

    rospy.loginfo(
        f' {GREEN} {BOLD} {UNDERLINE}  Image Feed Recieved.... {END}')
    count_tom = Int16()

    while not rospy.is_shutdown():
        if (type(img) == np.ndarray) and (type(dpt) == np.ndarray):
            try:
                img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                mask = cv2.inRange(img2, redLower, redUpper)
                mask = cv2.erode(mask, None, iterations=1)
                mask = cv2.dilate(mask, None, iterations=6)

                contours, _ = cv2.findContours(mask, cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_NONE)
                count = 0
                contours = list(contours)

                for contour in contours:
                    (x, y), _ = cv2.minEnclosingCircle(contour)
                    x = int(x)
                    y = int(y)

                    text = 'Tomato'
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    org = (x, y)
                    fontScale = 0.5
                    color = (255, 0, 0)
                    thickness = 1

                    depth = dpt[y, x]

                    # World Coordinates
                    X = depth * (x - cx) / f
                    Y = depth * (y - cy) / f
                    Z = depth

                    if Z < 1:  # Publish the transform only if the range is within the limit.
                        # Frame Id: camera_depth_frame2
                        t = geometry_msgs.msg.TransformStamped()
                        t.header.stamp = rospy.Time.now()
                        t.header.frame_id = 'camera_depth_frame2'
                        t.child_frame_id = "obj" + str(count)

                        # putting world coordinates coordinates as viewed for depth frame
                        t.transform.translation.x = X
                        t.transform.translation.y = Y
                        t.transform.translation.z = Z

                        # not extracting any orientation thus orientation is (0, 0, 0)
                        q = tf_conversions.transformations.quaternion_from_euler(
                            0, 0, 0)
                        t.transform.rotation.x = q[0]
                        t.transform.rotation.y = q[1]
                        t.transform.rotation.z = q[2]
                        t.transform.rotation.w = q[3]

                        if (X != 0) and (Y != 0) and (Z != 0):
                            broadcaster.sendTransform(t)
                        count += 1
                    cv2.putText(img, text, org, font, fontScale, color,
                                thickness,
                                cv2.LINE_AA)  # Put the text on image frame
                    cv2.drawContours(img, contour, -1, (0, 255, 0), 3)

                cv2.imshow("RGB IMAGE", img)

                cv2.waitKey(1)
                try:
                    if Z < 1:
                        count_tom.data = len(contours)
                        count_feed.publish(
                            count_tom
                        )  # Updates the manipulation node about number of tomatoes.

                except UnboundLocalError:
                    count_tom.data = 0
                    count_feed.publish(count_tom)

            except KeyboardInterrupt:
                print('Shutting Down')
                cv2.destroyAllWindows()
                break
            except:
                raise

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('perception',
                    anonymous=True)  # "perception" node initilization

    main()