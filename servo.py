#!/usr/bin/env python
import roslib
roslib.load_manifest('ball_tracking')
import sys
import rospy
import cv2
import numpy as np
import serial
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


ser = serial.Serial('/dev/ttyACM0')
pan = 1500
tilt = 1000


def callback(data):
    centers = data

def track():

    rospy.Subscriber("centers", List, callback)

    x = centers[0]
    y = centers[1]

    if x > 340 | x < 300:
      pan = pan + (x-320)
      if pan >= 2000:
        pan = 2000
      elif pan <= 1000:
        pan = 1000
    else:
      pan = pan

    if y > 260 | y < 220:
      tilt = tilt + (y-240)
      if pan >= 1450:
        tilt = 1450
      elif pan <= 1000:
        tilt = 1000
    else:
      tilt = tilt

    pancom = pan*4 & 0x7f
    pancom2 = (pan*4 >> 7) & 0x7f

    tiltcom = tilt*4 & 0x7f
    tiltcom2 = (tilt*4 >>7) & 0x7f

    #print pancom, pancom2
    #print tiltcom, tiltcom2

    panpos = bytearray([132,1,pancom,pancom2])
    tiltpos = bytearray([132,0,tiltcom,tiltcom2])

    ser.write(panpos)
    ser.write(tiltpos)

def main(args):

    rospy.init_node('tracker', anonymous=True)



if __name__ == '__main__':
    main(sys.argv)
