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

find_package(depthai_cmake REQUIRED)

add_ros_messages(
  PROJECT_NAME ${PROJECT_NAME}
  MESSAGES BoundingBox.msg Object.msg Objects.msg AutoFocusCtrl.msg
  SERVICES TriggerNamed.srv
  PKG_DEPS std_msgs sensor_msgs
)
