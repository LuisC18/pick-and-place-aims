#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians
from std_msgs.msg import *
from moveit_commander.conversions import pose_to_list
from motoman_msgs.srv import ReadSingleIO, WriteSingleIO
from geometry_msgs.msg import Pose

## Quaternion Tools
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#from robot_support import moveManipulator
from robot_support import *

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # global a 
    
    x = data.position.x
    y = data.position.y
    # a = data.position.x
    # b = data.position.y
    z = data.orientation.z
    
    
def listener():
    with open("/home/martinez737/ws_pick_camera/src/pick-and-place/scripts/Coordinate-angle.txt", "r") as f:
        file_content = f.read()
        fileList = file_content.splitlines()
        xC = float(fileList[0])/100
        yC = float(fileList[1])/100

        pose_goal = [0,0,0,0,0,0]
        pose_goal[0] = xC-0.1
        pose_goal[1] = yC+0.5
        pose_goal[2] = 0.2

        x_angle = radians(-180)
        y_angle = radians(45)
        z_angle = radians(float(fileList[2])-45)

        pose_goal[3]=x_angle
        pose_goal[4]=y_angle
        pose_goal[5]=z_angle
        return pose_goal

def main():
  try:
   
   ######
    pose_goal = listener()
    print(pose_goal)
    # print('x position:', a)
    # print('y-position:',b)
    # print('z-orientation:',c)
    
    #set velocity of motion
    rc = moveManipulator('bot_mh5l')
    rc.set_vel(0.1)
    rc.set_accel(0.1)

    #starting position and open gripper
    # raw_input('Go to All Zeroes <enter>')
    # rc.goto_all_zeros()
    # rc.send_io(0)

    #for simulation
    #raw_input('Add cube <enter>')
    #rc.add_object()

    raw_input('Begin Pick and Place <enter>')

    # move to object position
    # pose [pos: x, y, z, axes:x y z w]
    #pose_lower = [xC-0.08,yC+0.37,0.015,0,1,0,0]
    rc.goto_Quant_Orient(pose_goal)

    # #grasp object
    # rc.send_io(1)
    # rc.attach_object()

    # #raise object
    # pose_higher = [xC-0.08,-yC-0.37,.815,0,1,0,0]
    # rc.goto_Quant_Orient(pose_higher)

    # #lower object
    # pose_lower = [xC-0.08,-yC-0.37,0.015,0,1,0,0]
    # rc.goto_Quant_Orient(pose_lower)

    # #release object
    # rc.send_io(0)
    # rc.detach_object()
    #raw_input('Return to ztart <enter>')
    #return to all zeros
    #rc.goto_all_zeros()
    
    #rc.remove_object()

  except rospy.ROSInterruptException:
    exit()
  except KeyboardInterrupt:
    exit()

if __name__ == '__main__':
  main()