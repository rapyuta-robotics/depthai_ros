cmake_minimum_required(VERSION 3.5)

project(depthai_ros_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

################################################
# Compile in ROS
################################################
find_package(catkin QUIET)
if($ENV{ROS_VERSION} STREQUAL 1)
  message(WARNING "Compile ${PROJECT_NAME} as a ROS1 package")
  find_package (catkin REQUIRED COMPONENTS
    message_generation
    sensor_msgs
    std_msgs
  )

  ## Generate messages in the 'msg' folder
  add_message_files (
    FILES
    BoundingBox.msg
    Object.msg
    Objects.msg
    AutoFocusCtrl.msg
  )

  ## Generate services in the 'srv' folder
  add_service_files (
    FILES
    TriggerNamed.srv
  )

  ## Generate added messages and services with any dependencies listed here
  generate_messages (
    DEPENDENCIES
    sensor_msgs std_msgs
  )

  # catkin specific configuration
  catkin_package (
    CATKIN_DEPENDS message_runtime sensor_msgs std_msgs
  )
  RETURN()
else()
  message(VERBOSE "ROS version is $ENV{ROS_VERSION}, skipping ROS1 ${PROJECT_NAME}")
endif()

################################################
# Compile in ROS 2
################################################
if($ENV{ROS_VERSION} STREQUAL 2)
  message(WARNING "Compile ${PROJECT_NAME} as a ROS2 package")
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/BoundingBox.msg"
    "msg/Object.msg"
    "msg/AutoFocusCtrl.msg"
    "msg/Objects.msg"
    "msg/RegionOfInterestStamped.msg"
  )

  set(srv_files
    "srv/TriggerNamed.srv"
  )

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${srv_files}
    ${msg_files}
    DEPENDENCIES
      builtin_interfaces
    ADD_LINTER_TESTS
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
else()
  message(VERBOSE "ROS version is $ENV{ROS_VERSION}, skipping ROS2 ${PROJECT_NAME}")
endif()
