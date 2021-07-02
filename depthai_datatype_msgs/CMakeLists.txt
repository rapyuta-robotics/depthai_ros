cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

project (depthai_datatype_msgs VERSION 1.0.0)

find_package (catkin REQUIRED COMPONENTS
  message_generation
  depthai_common_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files (
  FILES
  ImgDetection.msg
  RawImgDetections.msg

  TrackingStatus.msg
  Tracklet.msg
  RawTracklets.msg

  TensorInfoDataType.msg
  TensorInfoStorageOrder.msg
  TensorInfo.msg
  RawNNData.msg

  RawImgFrameType.msg
  RawImgFrameSpecs.msg
  RawImgFrame.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages (
  DEPENDENCIES
  depthai_common_msgs
)

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
