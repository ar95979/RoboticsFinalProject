#!/usr/bin/env python
import rospy
import numpy as np
from PIL import Image as image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy
import tf
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import sys

color = "null"
contourLength = 0

def callbackColor(msg):
  #convert ROS image to CV image
  im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1) 
  img = image.fromarray(im, 'RGB')
  
  hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
  
  #color thresholds
  lower_red = numpy.array([255, 85, 0])
  upper_red = numpy.array([100, 0, 0])
  
  lower_green = numpy.array([180, 255, 0])
  upper_green = numpy.array([0, 60, 0])
  
  lower_blue = numpy.array([0, 115, 255])
  upper_blue = numpy.array([0, 0, 70])
  
  maskRed = cv2.inRange(hsv, lower_red, upper_red)
  maskGreen = cv2.inRange(hsv, lower_green, upper_green)
  maskBlue = cv2.inRange(hsv, lower_blue, upper_blue)

  cv2.morphologyEx(maskRed, cv2.MORPH_CLOSE, numpy.ones((11,11)))
  cv2.morphologyEx(maskGreen, cv2.MORPH_CLOSE, numpy.ones((11,11)))
  cv2.morphologyEx(maskBlue, cv2.MORPH_CLOSE, numpy.ones((11,11)))
    
  contoursRed = cv2.findContours(maskRed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  contoursGreen = cv2.findContours(maskGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  contoursBlue = cv2.findContours(maskBlue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

  contourLength = len(contoursRed)
  
  #check if color found in image
  #check red
  if contourLength > 1:
    print("red")
    color = 3

  contourLength = len(contoursGreen)
  
  #check green
  if contourLength > 1:
    print("green")
    color = 2

  contourLength = len(contoursBlue)
  
  #check blue
  if contourLength > 1:
    print("blue")
    color = 1
  else:
    print("no color")
    color = 0
  
def main():
  #script gets a single image from topic and displays it fro image manipulation
  rospy.init_node('image', anonymous=True) 
  
  rospy.Subscriber("/realsense/color/image_raw", Image, callbackColor)
  rospy.spin()

if __name__=="__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
