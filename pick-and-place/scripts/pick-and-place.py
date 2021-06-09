#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from motoman_msgs.srv import ReadSingleIO, WriteSingleIO

## Quaternion Tools
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#from robot_support import moveManipulator
from any_position_grasp import *

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber('positionData',String, callback)
    rospy.spin()
def main():
  try:
    listener()
    #set velocity of motion
    rc = moveManipulator('bot_mh5l')
    rc.set_vel(0.1)
    rc.set_accel(0.1)

    #starting position and open gripper
    raw_input('Go to All Zeroes <enter>')
    rc.goto_all_zeros()
    rc.send_io(0)

    #for simulation
    #raw_input('Add cube <enter>')
    #rc.add_object()

    raw_input('Begin Pick and Place <enter>')

    # move to object position
    # pose [pos: x, y, z, axes:x y z w]
    pose_lower = [0,0.6,0.015,0,1,0,0]
    rc.goto_Quant_Orient(pose_lower)

    #grasp object
    rc.send_io(1)
    rc.attach_object()

    #raise object
    pose_higher = [0,0.6,.815,0,1,0,0]
    rc.goto_Quant_Orient(pose_higher)

    #lower object
    pose_lower = [0,0.6,0.015,0,1,0,0]
    rc.goto_Quant_Orient(pose_lower)

    #release object
    rc.send_io(0)
    rc.detach_object()

    #return to all zeros
    rc.goto_all_zeros()
    
    #rc.remove_object()

  except rospy.ROSInterruptException:
    exit()
  except KeyboardInterrupt:
    exit()

if __name__ == '__main__':
  main()