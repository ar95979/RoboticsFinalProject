#!/usr/bin/env python                                                                                    
import rospy
import math
import time
from tf import transformations
import rosservice
import numpy as np
import cv2
from PIL import Image as image
from cv_bridge import CvBridge, CvBridgeError
import numpy
import tf
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import sys

color = 0

X =  [2,-1.5,-2.8,-2.7,-0.4]
Y = [2,1,1.8,3.2,5.7]

contourLength = 0

def callbackColor(msg):
    global color
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
        color = 0

def main():
    global X
    global Y
    global color

    rospy.init_node('project', anonymous=True) 
    #subscribers
    rospy.Subscriber("/realsense/color/image_raw", Image, callbackColor)
    rospy.spin()    
    
    #while loop to fetch all 5 waypoints from the server and do its thing
    first = True
    count = 0
    #get the first waypoint
    coorX = 0
    coorY = 0

    dict = rosservice.call_service("/Final_ints", [True,False,0,0,0])[1]
    if(dict.success):
        coorX = dict.waypointx
        coorY = dict.waypointy
    else:
        print("Something went wrong")
        pass

    while(count < 5):
        #########
        #Farah's script to move to waypoint w coorX and coorY
        #########
        print("Calling move script with coordinates (%.2f,%.2f)" % (coorX, coorY))
        
        #once move script is done, we will detect image
        time.sleep(2)

        #########
        #Chris' script to detect colour
        #########
        #should return the colour of the star and its coordinates in the robot frame
        objCoorX = X[count]
        objCoorY = Y[count]
        colour = Color[count]

        #########
        #transfrom lookingScript coordinates to world frame
        #
        #camera frame: camera_realsense_gazebo
        #world frame: odom
        #
        #objCoorX = transfromed X coordinate
        #objCoorY = transformed Y coordinate
        #########

        #communicate with the service with arguments: False True colour coorX coorY
        dict = rosservice.call_service("/Final_ints", [False,True,colour,objCoorX,objCoorY])[1]
        if(dict.success):
            print("object detected successfully")
            coorX = dict.waypointx
            coorY = dict.waypointy
            count += 1
        else:
            print("wrong colour, keep looking")

    #at the end of the while loop the coordinates have changed back to the origin coordinates
    print("Moving back to: (%.2f, %.2f)" % (coorX, coorY))
    #########
    #Farah's script to move to waypoint 2,2
    #########

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
