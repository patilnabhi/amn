#!/usr/bin/env python
import roslib
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
    
    blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    upper_red = np.array([6, 255, 245],np.uint8)
    lower_red = np.array([0, 90, 80],np.uint8)
    #hsv_lower_red = cv2.cvtColor(lower_red,cv2.COLOR_BGR2HSV)
    #hsv_upper_red = cv2.cvtColor(upper_red,cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_red, upper_red)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
 
    # only proceed if at least one contour was found
    if len(cnts) > 0:
      # find the largest contour in the mask, then use
      # it to compute the minimum enclosing circle and
      # centroid
      c = max(cnts, key=cv2.contourArea)
      ((x, y), radius) = cv2.minEnclosingCircle(c)
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
      # only proceed if the radius meets a minimum size
      if radius > 20:
        # draw the circle and centroid on the frame,
        # then update the list of tracked points
        cv2.circle(cv_image, (int(x), int(y)), int(radius),
          (0, 255, 255), 2)
        cv2.circle(cv_image, center, 5, (0, 0, 255), -1)

    cv2.imshow("Image window", cv_image))
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