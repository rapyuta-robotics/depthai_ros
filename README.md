# DepthAI ROS

This package facilitates the integration of DepthAI and compatible hardware with ROS. List of compatible hardwares can be found [here](https://docs.luxonis.com/en/latest/). This package is based on the Gen 1 API for DepthAI and provides an interface for ROS (not ROS2).

This meta-package contains the following packages:
* depthai_ros_driver
* depthai_ros_msgs
* node_interface

# Setup
## Requirements
Though this metapackage might work on other systems, it has only been tested on:
* Ubuntu 18.04 LTS
* Ubuntu 20.04 LTS

A list of pre-requisites:
* C++ compiler (recommended: `g++` or `clang` with at least C++17 support)
* CMake (3.10.2 or higher)
* Boost (1.65.1 or higher)
* OpenCV (3.2 or higher)
* ROS (melodic or higher)
* ROS Packages (see `rospack deps depthai_ros_driver`)

## Grab the source code
To use `depthai_ros` metapackage, clone this repo with submodules:
```
$ git clone --recursive https://github.com/rapyuta-robotics/depthai_ros.git
```
For older versions of git, one of the following processes might be needed:
* Use `--recurse-submodules` instead of `--recursive`
* Clone without any flags and then grab the submodules using `git submodule update --init --recursive`
* Prepare MobileNetSSD compatible model (blob and json file). Once issue #5 is resolved, this will not be necessary for a default model. Due to limitations of Gen1 API used in the ROS driver, a model is required to run it.

# Usage
0. Compile using `cmake build depthai_ros` or your preferred `cmake_build_isolated` or `colcon` process. `catkin_build` will **not build** the package successfully.
1. Launch the node or nodelet by appropriately choosing the correct launch file and parameters:
    ```
    roslaunch depthai_ros_driver depthai_{node,nodelet}.launch
    ```
3. Custom models can be provided using `blob_file` and `blob_file_config` arguments to the launch file. If an argument `A` needs value `B`, the syntax is `roslaunch <launch_file> A:=B`
2. Get the relevant details using:
    ```
    $ rostopic list /depthai
    $ rosservice list /depthai
    $ rosparam list /depthai
    ```

# Details

## `depthai_ros_msgs`
This package contains all the custom messages and services used by the `depthai_ros_driver` API. While we tried to use the standard messages where possible, due to the unique hardware, it wasn't possible to stick with the standard messages everywhere.

## `depthai_ros_driver`
This package consists of a node and a nodelet version of the ROS driver. It also contains the launch files required for an easy use.

## `node_interface`
This package contains some utilities used to reduce code duplication between the node and the nodelet version in `depthai_ros_driver`.
