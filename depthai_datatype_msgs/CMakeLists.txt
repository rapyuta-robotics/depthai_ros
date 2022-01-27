cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

project (depthai_datatype_msgs VERSION 1.0.0)

find_package (catkin REQUIRED COMPONENTS
  message_generation
  depthai_common_msgs
  depthai_hunter_deps
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files (
  FILES
  ImgDetection.msg
  RawImgDetections.msg
  ImgDetectionsStamped.msg

  # TrackingStatus.msg
  Tracklet.msg
  RawTracklets.msg

  # TensorInfoDataType.msg
  # TensorInfoStorageOrder.msg
  TensorInfo.msg
  RawNNData.msg

  # RawImgFrameType.msg
  RawImgFrameSpecs.msg
  RawImgFrame.msg

  # # IMU data type not converted due to duplicate field names
  # IMUReportAccuracy.msg
  # IMUReport.msg
  # IMUReportAccelerometer.msg
  # IMUReportGyroscope.msg
  # IMUReportMagneticField.msg
  # # IMUReportRotationVectorWAcc.msg
  # # IMUPacket.msg
  # # RawIMUData.msg

  # RawCameraControlAntiBandingMode.msg
  # RawCameraControlAutoFocusMode.msg
  # RawCameraControlAutoWhiteBalanceMode.msg
  # RawCameraControlCaptureIntent.msg
  # RawCameraControlCommand.msg
  # RawCameraControlControlMode.msg
  # RawCameraControlEffectMode.msg
  # RawCameraControlSceneMode.msg
  ManualExposureParams.msg
  RegionParams.msg
  RawCameraControl.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages (
  DEPENDENCIES
  depthai_common_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package (
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime depthai_hunter_deps
)

include_directories (SYSTEM
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
