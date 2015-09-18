#!/usr/bin/env python
import roslib
import serial
roslib.load_manifest('ball_tracking')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    # rospy.init_node('image_converter')

    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    (rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)
    
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    upper_red = np.array([9, 255, 245],np.uint8)
    lower_red = np.array([0, 90, 80],np.uint8)
    #hsv_lower_red = cv2.cvtColor(lower_red,cv2.COLOR_BGR2HSV)
    #hsv_upper_red = cv2.cvtColor(upper_red,cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_red, upper_red)

    res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

    output = res.copy()
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 2)

    # detect circles in the image
    circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 3, 1000)
 
    # ensure at least some circles were found
    if circles is not None:
      # convert the (x, y) coordinates and radius of the circles to integers
      circles = np.round(circles[0, :]).astype("int")

      # loop over the (x, y) coordinates and radius of the circles
      for (x, y, r) in circles:
        # draw the circle in the output image, then draw a rectangle
        # corresponding to the center of the circle
        cv2.circle(output, (x, y), r, (0, 255, 0), 4)
        cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
        return x, y

    cv2.imshow("Image window", np.hstack([cv_image, res, output]))
    # cv2.imshow('mask',mask)
    # cv2.imshow('res',res)
    # cv2.imshow('output',output)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)