cmake_minimum_required(VERSION 2.8.3)

project(lab_launcher)

find_package(catkin REQUIRED)

######################################
## Catkin Specific Configuration

catkin_package()

######################################
## Install

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
