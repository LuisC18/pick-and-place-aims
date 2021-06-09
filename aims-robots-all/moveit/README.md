<img src="http://moveit.ros.org/assets/images/moveit2_logo_black.png" alt="MoveIt! Logo" width="200"/>

# MoveIt Planning Packages
Packages developed to be used with the MoveIt Motion Planning Framework
Reference below and https://github.com/ros-planning/moveit for usage information.

- [Overview of MoveIt!](http://moveit.ros.org)
- [Documentation](http://moveit.ros.org/documentation/)

## Building

### Required Dependencies
_Robot Support Packages_
Must have appropriate robot_support packages loaded into the same catkin workspace to use. All moveit packages have a <run_dependency> on the correlated robot_support package.

_Moveit Support Package_
Must have moveit support package installed. Follow instructions at [MoveIt Installation Instructions](http://moveit.ros.org/install/).

### On newer (or older) versions of ROS

Building the packages on newer (or older) versions of ROS is in most cases possible and supported. For example: building the packages in this repository on Ubuntu Xenial/ROS Kinetic or Ubuntu Bionic/ROS Melodic systems is supported. This will require creating a Catkin workspace, cloning this repository, installing all required dependencies and finally building the workspace.

### Catkin tools

It is recommended to use [catkin_tools][] instead of the default [catkin][] when building ROS workspaces. `catkin_tools` provides a number of benefits over regular `catkin_make` and will be used in the instructions below. All packages can be built using `catkin_make` however: use `catkin_make` in place of `catkin build` where appropriate.



## Installation and usage

Refer to [Working With ROS-Industrial Robot Support Packages][] for information on how to use the files provided by the robot support and MoveIt configuration packages. See also the other pages on the [ROS wiki][].

Refer to the [tutorials][] for information on installation and configuration of the controller-specific software components.



[ROS-Industrial]: http://wiki.ros.org/Industrial
[ROS wiki]: http://wiki.ros.org/motoman
[motoman_experimental]: https://github.com/ros-industrial/motoman_experimental
[subversion repository]: https://github.com/ros-industrial/swri-ros-pkg
[Catkin workspace]: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
[catkin]: http://wiki.ros.org/catkin
[catkin_tools]: https://catkin-tools.readthedocs.io/en/latest
[Working With ROS-Industrial Robot Support Packages]: http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages
[tutorials]: http://wiki.ros.org/motoman_driver/Tutorials
