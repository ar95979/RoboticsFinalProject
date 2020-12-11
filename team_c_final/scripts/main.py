#!/usr/bin/env python                                                                                    
import rospy
import math
import time
from tf import transformations
import rosservice

Color = [2,1,1,3,3]
X =  [2,-1.5,-2.8,-2.7,-0.4]
Y = [2,1,1.8,3.2,5.7]

def main():
    global X
    global Y
    global Color
    
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

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
