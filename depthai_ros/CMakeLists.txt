cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

project (depthai_ros VERSION 0.1.0)

if($ENV{ROS_VERSION} STREQUAL 1)
    find_package (catkin REQUIRED)
    catkin_metapackage ()
endif()
