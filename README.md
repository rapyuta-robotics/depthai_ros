# DepthAI ROS

This package facilitates the integration of DepthAI and compatible hardware with ROS. List of compatible hardwares can be found [here](https://docs.luxonis.com/en/latest/)

This meta-package contains the following packages:
* depthai_ros_driver
* depthai_ros_msgs
* node_interface

## `depthai_ros_msgs`
This package contains all the custom messages and services used by the `depthai_ros_driver` API. While we tried to use the standard messages where possible, due to the unique hardware, it wasn't possible to stick with the standard messages everywhere.

## `depthai_ros_driver`
This package consists of a node and a nodelet version of the ROS driver. It also contains the launch files required for an easy use.

## `node_interface`
This package contains some utilities used to reduce code duplication between the node and the nodelet version in `depthai_ros_driver`.
