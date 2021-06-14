#!/usr/bin/env python

import sys
import copy
import rospy
import numpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi, radians, sqrt
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
        z_angle = radians(float(fileList[2])-90)

        t= [math.cos(z_angle),math.sin(z_angle),0,
        math.sin(z_angle),-math.cos(z_angle),0,
        0,0,-1]

        w = sqrt(t[0]+t[4]+t[8]+1)/2
        x = sqrt(t[0]-t[4]-t[8]+1)/2
        y = sqrt(-t[0]+t[4]-t[8]+1)/2
        z = sqrt(-t[0]-t[4]+t[8]+1)/2
        a = [w,x,y,z]
        m = a.index(max(a))
        if m == 0:
            x = (t[7]-t[5])/(4*w)
            y = (t[2]-t[6])/(4*w)
            z = (t[3]-t[1])/(4*w)
        if m == 1:
            w = (t[7]-t[5])/(4*x)
            y = (t[1]+t[3])/(4*x)
            z = (t[6]+t[2])/(4*x)
        if m == 2:
            w = (t[2]-t[6])/(4*y)
            x = (t[1]+t[3])/(4*y)
            z = (t[5]+t[7])/(4*y)
        if m == 3:
            w = (t[3]-t[1])/(4*z)
            x = (t[6]+t[2])/(4*z)
            y = (t[5]+t[7])/(4*z)
        b = [w,x,y,z]
        print(b)

        pose_goal = [0,0,0,0,0,0,0]
        pose_goal[0] = yC-0.07
        pose_goal[1] = -xC-0.37
        pose_goal[2] = 0.2


        # x_angle = radians(-180)
        # y_angle = radians(0)
        
        # #z_angle = radians(24)
        # print('Z angle <= radians(-1)',radians(-1))
        # print('Z angle: ',str(z_angle))

        

        # if z_angle <= radians(-1):
        #     print('Z angle <= radians(-1)',radians(-1))
        #     z_angle = radians(float(fileList[2])+135)
        #     print('Angle is negative.')
        # else:
        #     print('Z_angle > 0')
        #     z_angle = radians(float(fileList[2])-135)

        # Test 1: Object oriented at 0: Success
        # Test 2: Oriented at -45: success
        # Test 3: Orientated at -90: Success
        # Test 4: Orient at ~ +22 degrees Success
        # Test 5: Orient at -80 degrees:  Success
        # Using +90 degrees should work for most cases, now up to camera to properly recognize angle
        # pose_goal[3]= 0.990268
        # pose_goal[4]=-0.139173
        # pose_goal[5]=0
        # pose_goal[6]=-0.0000012228

        pose_goal[3]=b[1]
        pose_goal[4]=b[2]
        pose_goal[5]=b[3]
        pose_goal[6]=b[0]

        return pose_goal

def main():
  try:
   
   ######
    pose_goal = listener()
    print('posegoal:',pose_goal)
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
    raw_input('Return to ztart <enter>')
    #return to all zeros
    rc.goto_all_zeros()
    
    #rc.remove_object()

  except rospy.ROSInterruptException:
    exit()
  except KeyboardInterrupt:
    exit()

if __name__ == '__main__':
  main()