#!/usr/bin/env python

import numpy as np
import rospy
import imutils

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # flip camera image if upsidedown
        ZED_UPSIDEDOWN = True
        LINE_FOLLOWER = False
        rotated = image
        if (ZED_UPSIDEDOWN):
            rotated = imutils.rotate(image, 180)

        if (LINE_FOLLOWER):
            h = image_msg.height
            w = image_msg.width
            rotated = rotated[int(3*h/4.0):, 0:]
        bbox = cd_color_segmentation(rotated, "no_template")
        pixel_msg = ConeLocationPixel()
        # publishing middle x and bottom y of bbox
        if (bbox[1] == (0,0)):
            pixel_msg.u = -1
            pixel_msg.v = -1
        else:
            pixel_msg.u = (bbox[0][0] + bbox[1][0]) * 0.5
            if (LINE_FOLLOWER):
		h = image_msg.height
                pixel_msg.v = bbox[0][1] + int(3*h/4.0)
            else: 
                pixel_msg.v = bbox[1][1]
        self.cone_pub.publish(pixel_msg)
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
