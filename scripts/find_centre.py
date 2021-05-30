#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
import cv2
import random
import numpy as np
import sys

random.seed(12345)

class FindCentre:
    def __init__(self):
        rospy.init_node("find_centre_of_stairs")
        self.sub = rospy.Subscriber("/kinect/depth/image_raw", Image, self.callback)
        self.rate = rospy.Rate(1)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data)
            frame = frame /3
            #print(frame.shape)
            # cv2.imshow("Input Image", frame)

            output_frame = frame.copy()

            #retval, thresh_image = cv2.threshold(frame, 200, 255, cv2.THRESH_BINARY_INV)
            #cv2.imshow("Thresholded Image", thresh_image)

            #sobelx_edges = cv2.Sobel((frame*255).astype(np.uint8), cv2.CV_64F, 0, 1, 3)
            #cv2.imshow("Sobelx Edge detected", sobelx_edges)

            canny = cv2.Canny((frame*255).astype(np.uint8), 150, 225, apertureSize=3)
            #kernel = np.ones((5,5), np.uint8)
            #dilation = cv2.dilate(canny, kernel, iterations=1)
            # cv2.imshow("Canny View", canny)

            lines = cv2.HoughLinesP((canny*255).astype(np.uint8), 1, np.pi/2, 100, minLineLength=225, maxLineGap=50)

            for line in lines:
                x1, y1, x2, y2 = line[0]

                mid_point = ((x1+x2)//2,(y1+y2)//2)
                # print(mid_point)
                cv2.line(output_frame, (x1,y1), (x2,y2), (255,255,0), 2)
                cv2.circle(output_frame, mid_point, 10, (255,0,0), -1)

            output_frame = np.vstack((output_frame, canny))
            # print(output_frame.shape)
            # self.out.write(output_frame.astype(np.uint8))
            cv2.imshow("Output Image", output_frame)

            cv2.waitKey(60)

        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    find_centre = FindCentre()
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
