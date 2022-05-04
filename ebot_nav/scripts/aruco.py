#!/usr/bin/env python3

"""
Description: Obtaining TF of a ArUco marker using single sjcam camera
Algorithm: 
	Input: ROS topic for the RGB image
	Process: 
        - Subscribe to the image topic
		- Convert ROS format to OpenCV format
		- Detecting ArUco marker along with its ID
		- Applying perspective projection to calculate focal length
		- Extracting depth for each aurco marker
		- Broadcasting TF of each ArUco marker with naming convention as aruco1 for ID1, aruco2 for ID2 and so on.
	Output: TF of ArUco marker with respect to ebot_base
"""

import cv2 as cv
import numpy as np
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from std_msgs.msg import String

cv_image = None
img = None
name = "string"
cX = 0
cY = 0


def callback(data):
    global img
    img = data



def get_transforms(tf_id):
    '''
    Function used to return transforms for the tomato.
    '''

    tf_buffer = tf2_ros.Buffer() # Transformation Buffer
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        transform = tf_buffer.lookup_transform('sjcam_link', tf_id,
                                               rospy.Time(),
                                               rospy.Duration(3.0))
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        z = transform.transform.translation.z

        return [x,y,z]

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        return [0,0,0]
    
def main(args):
    previous = " "
    rospy.init_node('aruco_tf', anonymous=True)
    # subscribing to /ebot/camera1/image_raw topic which is the image frame of sjcam camera
    image_sub = rospy.Subscriber("/ebot/camera1/image_raw", Image, callback)
    aruco = rospy.Publisher("aruco",String, queue_size=10)
    aruco_msg = String()
    aruco_msg.data = "stop"
    while not rospy.is_shutdown():


        focal_length = 476.70308
        center_x = 400.5
        center_y = 400.5
        aruco_dimension = 0.1
        if type(img) == Image:
            try:
                bridge = CvBridge()
                frame = bridge.imgmsg_to_cv2(img, "bgr8")
                frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

                # load the dictionary that was used to generate the markers
                dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_7X7_1000)

                # initializing the detector parameters with default values
                parameters =  cv.aruco.DetectorParameters_create()

                # detect the markers in the frame
                corners, ids, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)
                id = ids
                if len(corners) > 0:
                    # Flatten the ArUco IDs list
                    ids = ids.flatten()
                    # loop over the detected ArUCo corners
                    for (markerCorner, markerID) in zip(corners, ids):
                        
                        
                        # extract the marker corners (which are always returned
                        # in top-left, top-right, bottom-right, and bottom-left
                        # order)
                        corners = markerCorner.reshape((4, 2))
                        (topLeft, topRight, bottomRight, bottomLeft) = corners
                        # convert each of the (x, y)-coordinate pairs to integers
                        topRight = (int(topRight[0]), int(topRight[1]))
                        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                        topLeft = (int(topLeft[0]), int(topLeft[1]))

                        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                        cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                        name = "aruco"+str(markerID)
  
                        print(cX)
                        if name != previous and previous != " ":
                            
                            print(cX in range(300,500))
                            if cX in range(300,500):
                                # aruco.publish(aruco_msg)
                                print(cX)
                                print(aruco_msg)
                        previous = name
                '''uncoment to view the visual of detection'''
                cv.imshow("frame", frame)
                cv.waitKey(1)
                
            except CvBridgeError as e:
                print(e)
    cv.destroyAllWindows()
        
if __name__ == '__main__':
    main(sys.argv)