#!/usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import math
import cv2
import rospy
from std_msgs.msg import *
import geometry_msgs.msg 

class detectRect(object):
  def __init__(self):
    super(detectRect,self).__init__()
    rospy.init_node('node_detectRectangle',anonymous=True)
    from geometry_msgs.msg import Pose
    self.sendPosOrient= rospy.Publisher('Coordinates/Angle', Pose, queue_size=10)
    self.pipeline = rs.pipeline()
    self.config = rs.config()

    # Get device product line for setting a supporting resolution
    self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
    self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
    self.device = self.pipeline_profile.get_device()
    self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

    self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if self.device_product_line == 'L500':
      self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
      self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    

    self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if self.device_product_line == 'L500':
      self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
      self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Starts streaming
    self.pipeline.start(self.config)

  def empty(self,a):
    pass
    # Determines whether coords and angle for object are consistent

  def isSame(self,index, xList, yList, angleList):
    j = 0
    countX = 0
    countY = 0
    countA = 0
    while (j < (index-1)):
      if (xList[j] >= (xList[j+1] - 1) and xList[j] <= (xList[j+1] + 1)):
        countX = countX + 1
      if (yList[j] >= (yList[j+1] - 1) and yList[j] <= (yList[j+1] + 1)):
        countY = countY + 1
      if (angleList[j] >= (angleList[j+1] - 3) and angleList[j] <= (angleList[j+1] + 3)):
        countA = countA + 1
      j = j + 1
    if (countX == 20 & countY == 20 & countA == 20):
      same = True
      return same
    else:
      same = False
      return same

    # Reorders the four points of the rectangle
  def reorder(self,points):
    try:
      NewPoints = np.zeros_like(points)
      points = points.reshape((4, 2))
      add = points.sum(1)
      NewPoints[0] = points[np.argmin(add)]
      NewPoints[3] = points[np.argmax(add)]
      diff = np.diff(points, axis=1)
      NewPoints[1] = points[np.argmin(diff)]
      NewPoints[2] = points[np.argmax(diff)]
      return NewPoints
    except:
      return []

  # Warps the Image
  def warpImg(self,img, points, wP, hP):
    pad = 20
    points = self.reorder(points)
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [wP, 0], [0, hP], [wP, hP]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    wint = int(wP)
    hint = int(hP)
    imgWarp = cv2.warpPerspective(img, matrix, (wint, hint))
    imgWarp = imgWarp[pad:imgWarp.shape[0] - pad, pad:imgWarp.shape[1] - pad]
    return imgWarp

  # Finds the distance between 2 points
  def findDis(self,pts1, pts2):
    x1 = float(pts1[0])
    x2 = float(pts2[0])
    y1 = float(pts1[1])
    y2 = float(pts2[1])
    dis = ((x2 - x1)**2 + (y2 - y1)**2)**(0.5)
    return dis

  # Draws x-y axis relative to object center and orientation
  def drawAxis(self,img, p_, q_, color, scale):
    p = list(p_)
    q = list(q_)

    angle = math.atan2(p[1] - q[1], p[0] - q[0])  #in radians
    hypotenuse = math.sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))

    # Lengthens the arrows by a factor of scale
    q[0] = p[0] - scale * hypotenuse * math.cos(angle)
    q[1] = p[1] - scale * hypotenuse * math.sin(angle)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)

    # Creates the arrow hooks
    p[0] = q[0] + 9 * math.cos(angle + math.pi / 4)
    p[1] = q[1] + 9 * math.sin(angle + math.pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)

    p[0] = q[0] + 9 * math.cos(angle - math.pi / 4)
    p[1] = q[1] + 9 * math.sin(angle - math.pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)

  # Gets angle of object
  def getOrientation(self,pts, img): 
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
      data_pts[i, 0] = pts[i, 0, 0]
      data_pts[i, 1] = pts[i, 0, 1]

    # Performs PCA analysis
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)

    # Stores the center of the object
    cntr = (int(mean[0, 0]), int(mean[0, 1]))
    # Draws the principal components
    cv2.circle(img, cntr, 3, (255, 0, 255), 2)
    p1 = (cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0],
          cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0])
    p2 = (cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0],
          cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0])
    self.drawAxis(img, cntr, p1, (255, 255, 0), 1)
    self.drawAxis(img, cntr, p2, (0, 0, 255), 5)
    angle = math.atan2(eigenvectors[0, 1], eigenvectors[0, 0])  # orientation in radians
    return angle, cntr, mean

  # Finds largest rectangular object
  def getContours(self,img, imgContour, minArea, filter):
    cThr = [100, 100]
    try:
      imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 1)
    except:
      imgBlur = cv2.GaussianBlur(img,(5,5),1)
    imgCanny = cv2.Canny(imgBlur, cThr[0], cThr[1])
    kernel = np.ones((5, 5))
    imgDilate = cv2.dilate(imgCanny, kernel, iterations=3)
    imgThre = cv2.erode(imgDilate, kernel, iterations=2)
    contours, hierarchy = cv2.findContours(imgThre, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    areaList = []
    approxList = []
    bboxList = []
    for i in contours:
      area = cv2.contourArea(i)
      if area > minArea:
        cv2.drawContours(imgContour, contours, 0, (255, 0, 0), 3)
        peri = cv2.arcLength(i, True)
        approx = cv2.approxPolyDP(i, 0.02 * peri, True)
        bbox = cv2.boundingRect(approx)
        if len(approx) == filter:
          areaList.append(area)
          approxList.append(approx)
          bboxList.append(bbox)
    if len(areaList) != 0:
      areaList= sorted(areaList, reverse=True)
    return areaList, approxList, bboxList
  def talker():
    #pub = rospy.Publisher('Coordinates/Angle',Pose, queue_size=10)
    # is float64 right for Python 2.7? 
    from geometry_msgs.msg import Pose
    print('Initialized Node')
    pub = rospy.Publisher('cameraPose',Pose, queue_size=10) #TODO: couldn't get latch=True to work. Looping instead
    rospy.init_node('cameraPose', anonymous=False)
    rate = rospy.Rate(1) # 10hz

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = xC
    pose_goal.position.y = yC
    pose_goal.orientation.z = pub_angle


    # Publish node
    while not rospy.is_shutdown():
        #rospy.loginfo(message)
        pub.publish(pose_goal)
        rate.sleep()