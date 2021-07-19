cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

project (depthai_common_msgs VERSION 1.0.0)

find_package (catkin REQUIRED COMPONENTS
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files (
  FILES
  # CameraBoardSocket.msg
  # CameraImageOrientation.msg
  # CameraModel.msg
  ChipTemperature.msg
  CpuUsage.msg
  # DetectionNetworkType.msg
  MemoryInfo.msg
  Point2f.msg
  Point3f.msg
  # ProcessorType.msg
  Rect.msg
  RotatedRect.msg
  Size2f.msg
  Timestamp.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages ()

###################################
## catkin specific configuration ##
###################################
catkin_package (
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
