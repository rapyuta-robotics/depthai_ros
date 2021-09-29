cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

project (depthai_ros_msgs VERSION 1.0.0)

find_package (catkin REQUIRED COMPONENTS
  message_generation
  depthai_datatype_msgs
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files (
  FILES
  ImgDetectionsStamped.msg
  NNStamped.msg
)

## Generate services in the 'srv' folder
add_service_files (
  FILES
  TriggerNamed.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages (
  DEPENDENCIES
  depthai_datatype_msgs sensor_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package (
  CATKIN_DEPENDS message_runtime
  depthai_datatype_msgs sensor_msgs std_msgs
)
