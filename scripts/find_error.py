#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class LineFollower:
    def __init__(self):
        rospy.init_node("find_error_in_linefollower")
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        self.param = dict()
        self.sub = rospy.Subscriber("/bottom_camera/bottom_camera/image", Image, self.callback)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        self.intg = self.lastError = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        error = self.process(cv_image)
        balance = self.pid(error)

    def process(self, image):
        width, height, channel = image.shape

        roi_w = int(self.param["ROI_W"] * width)
        roi_h = int(self.param["ROI_H"] * height)
        roi_y = int(self.param["ROI_Y"] * height)

        tl = ((width - roi_w)//2, roi_y)
        br = ((width + roi_w)//2, roi_y + roi_h)

        roi = image[tl[1]:br[1], tl[0]:br[0]]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        retval, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.rectangle(image, tl, br, (0,0,255), 2)
        cv2.drawContours(roi, contours, -1, (0,255,255), 3)
        cv2.line(image, (width//2, tl[1]), (width//2, br[1]), (255,0,0), 2)

        for cnt in contours:
            mu = cv2.moments(cnt)
            mc = (int(mu["m10"]/(mu["m00"] + 1e-5)), int(mu["m01"]/(mu["m00"] + 1e-5)))

            cv2.circle(roi, mc, 5, (0,0,255), -1)
            cv2.circle(roi, (roi_w//2, mc[1]), 5, (255,0,0), -1)
            cv2.line(roi, mc, (roi_w//2, mc[1]), (255,0,255), 3)
            cv2.imshow("Image", image)
            cv2.waitKey(60)
            print(width//2, mc)
            error = width//2 - mc[0]
            return error

        cv2.imshow("Image", image)
        cv2.waitKey(60)
        return 0

    def pid(self, error):
        print(error)
        prop = error
        self.intg -= error
        diff = error - self.lastError
        self.lastError = error
        balance = self.param["KP"] * prop + self.param["KI"] * self.intg + self.param["KD"] * diff
        return balance

if __name__ == "__main__":
    linefollower = LineFollower()
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
