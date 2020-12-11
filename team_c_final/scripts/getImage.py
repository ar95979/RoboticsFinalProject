#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from PIL import Image as image

def main():
  #script gets a single image from topic and displays it fro image manipulation
  rospy.init_node('image', anonymous=True) 
  
  msg = rospy.wait_for_message("/realsense/color/image_raw", Image)
  print(msg)
  im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1) 
  img = image.fromarray(im, 'RGB')
  img.save('my.png')
  img.show()

if __name__=="__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
