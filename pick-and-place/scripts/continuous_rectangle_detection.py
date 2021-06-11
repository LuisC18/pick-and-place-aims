#!/usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import math
import cv2
import rospy
from std_msgs.msg import *
import geometry_msgs.msg 

from rectangle_support import *
rect = detectRect()


# Makes list for coords and angle
index = 0
xList = []
yList = []
angleList = []
# Scaled width and height of the paper
scale = 2.5
hP = 216 * scale  # in mm
wP = 279.4 * scale  # in mm
loop =True

    
#def talker():
  


def main():
  index=0
  try:
    while loop:
      frames = rect.pipeline.wait_for_frames()
      depth_frame = frames.get_depth_frame()
      color_frame = frames.get_color_frame()
      if not depth_frame or not color_frame:
        continue

      # Converts images to numpy arrays
      depth_image = np.asanyarray(depth_frame.get_data())
      color_image = np.asanyarray(color_frame.get_data())

      # Applys colormap on depth image (image must be converted to 8-bit per pixel first)
      depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

      depth_colormap_dim = depth_colormap.shape
      color_colormap_dim = color_image.shape

      # If depth and color resolutions are different, resize color image to match depth image for display
      if depth_colormap_dim != color_colormap_dim:
        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                         interpolation=cv2.INTER_AREA)
        images = np.hstack((resized_color_image, depth_colormap))
      else:
        images = np.hstack((color_image, depth_colormap))

      # Show images
      cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
      imgCont = color_image.copy()

      # Gets Contour of Paper
      a, b, c = rect.getContours(color_image, imgCont, minArea = 3000, filter=4)
      cv2.imshow('RealSense',imgCont)

      if len(a) != 0:
        biggest = b[0]

        # Warps Image
        imgWarp = rect.warpImg(imgCont, biggest, wP, hP)
        imgCont2 = imgWarp.copy()

        # Gets Contour of Block
        a2, b2, c2 = rect.getContours(imgWarp, imgCont2, minArea =20, filter = 4)
        if len(b2) != 0:
          # Finds biggest contour
          biggest2 = b2[0]
          cv2.polylines(imgCont2,b2,True,(0,255,0),2)
          nPoints = rect.reorder(biggest2)
          if len(nPoints) != 0:
            # Locates center point, distance to edge of paper and finds angle
            NewWidth = round(rect.findDis(nPoints[0][0]//scale, nPoints[1][0]//scale)/10,3)   #in cm
            NewHeight = round(rect.findDis(nPoints[0][0]//scale, nPoints[2][0]//scale)/10,3)  #in cm
            cv2.arrowedLine(imgCont2, (nPoints[0][0][0], nPoints[0][0][1]),(nPoints[1][0][0], nPoints[1][0][1]),
                        (255,0,255),3,8,0, 0.05)
            cv2.arrowedLine(imgCont2, (nPoints[0][0][0], nPoints[0][0][1]), (nPoints[2][0][0], nPoints[2][0][1]),
                        (255,0,255),3,8,0,0.05)
            x,y,w,h = c2[0]

            angle, cntr, mean = rect.getOrientation(nPoints, imgCont2)
            result_angle = int(np.rad2deg(angle)) # in deg
            xC = round((wP/(10*scale)) - (round(rect.findDis((cntr[0], cntr[1]), (0, cntr[1])) / (10*scale), 3)),3)  # in cm
            yC = round((hP/(10*scale)) - (round(rect.findDis((cntr[0], cntr[1]), (cntr[0], 0)) / (10*scale), 3)+2),3)   # in cm
            
            # Makes List for coordinates and angle
            xList.append(xC)
            yList.append(yC)
            angleList.append(result_angle)
            index = index + 1

            # Draws arrows and puts text on image
            cv2.arrowedLine(imgCont2, (cntr[0], cntr[1]), (cntr[0],1000), (0, 255, 0), 2, 8, 0, 0.05)
            cv2.arrowedLine(imgCont2, (cntr[0],cntr[1]), (1000, cntr[1]), (0, 255,0),2,8,0,0.05)
            center = [NewWidth / 2, NewHeight / 2]
            cv2.putText(imgCont2, '{}cm'.format(NewWidth),(x+30, y-10), cv2.FONT_HERSHEY_PLAIN, 0.75, (0,0,0),1)
            cv2.putText(imgCont2, '{}cm'.format(NewHeight),(x-70, y+h//2), cv2.FONT_HERSHEY_PLAIN, 0.75, (0,0,0),1)
            label = "  Rotation Angle: " + str(int(np.rad2deg(angle)) + 90) + " degrees"
            textbox = cv2.rectangle(imgCont2, (cntr[0], cntr[1] - 25), (cntr[0] + 250, cntr[1] + 10),
                                    (255, 255, 255), 1)
            cv2.putText(imgCont2, label, (cntr[0], cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1,
                        cv2.LINE_AA)
            cv2.imshow('Warp',imgCont2)

            # # Checks whether the same coords and angle are being detected consistently
            # if (index >= 19):
            #   print('counting')
            #   same = rect.isSame(index, xList, yList, angleList)
            #   pub = rospy.Publisher('cameraPose',Pose, queue_size=10)
            # rospy.init_node('cameraPose', anonymous=False)if (same == True):
            #     print('********* RESULTS ***************')
            #     print('Angle is ' + str(result_angle) + ' degrees [CCW Positive]')
            #     print('Coordinate of center is (' + str(xC) + ' , ' + str(yC) + ') cm')
            pub_angle = int(np.rad2deg(result_angle))
            #     rect.talker()
            #     #rospy.sleep(5)
            from geometry_msgs.msg import Pose
            # pub = rospy.Publisher('cameraPose',Pose, queue_size=10)
            # rospy.init_node('cameraPose', anonymous=False)
            rate = rospy.Rate(1) # 10hz

            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = xC
            pose_goal.position.y = yC
            pose_goal.orientation.z = pub_angle

            # Publish node
            while not rospy.is_shutdown():
                #rospy.loginfo(message)
                rect.sendPosOrient.publish(pose_goal)
                rate.sleep()

      cv2.waitKey(1)


  except rospy.ROSInterruptException:
    exit()
  except KeyboardInterrupt:
    exit()

if __name__ == '__main__':
  main()