cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

set(CMAKE_CXX_STANDARD 17)

get_filename_component(
  DEPTHAI_CORE
  "${CMAKE_CURRENT_LIST_DIR}/../external/depthai-core"
  REALPATH
)

include("${DEPTHAI_CORE}/cmake/HunterGate.cmake")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(HUNTER_CONFIGURATION_TYPES "Release"
  CACHE
  STRING "Hunter dependencies list of build configurations"
)

HunterGate(
  URL "https://github.com/cpp-pm/hunter/archive/v0.23.258.tar.gz"
  SHA1 "062a19ab13ce8dffa9a882b6ce3e43bdabdf75d3"
  FILEPATH ${DEPTHAI_CORE}/cmake/Hunter/config.cmake # Add depthai-core config  (hunter limitation)
)

project(depthai_ros_driver VERSION 0.1.0 LANGUAGES CXX)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(COMMON_PKG_DEPS
  camera_info_manager
  cv_bridge
  depthai_ros_msgs
  image_transport
  sensor_msgs
  std_msgs
)

if($ENV{ROS_VERSION} STREQUAL 1)
  set (PKG_DEPS
    ${COMMON_PKG_DEPS}
    nodelet
    roscpp
    node_interface
  )
else()
  set (PKG_DEPS
    ${COMMON_PKG_DEPS}
    rclcpp
    rclcpp_components
  )
endif()

find_package(depthai_cmake REQUIRED)

# A bug in the CMake of the included external project depthai-core clobbers the
# paths used to find ROS packages, so we have to search for them first and then
# add the external project
find_dependent_packages(${PKG_DEPS})

add_subdirectory(${DEPTHAI_CORE} depthai-core)

add_ros_node(depthai_ros_driver
  SKIP_PACKAGE_SEARCH
  PROJECT_NAME ${PROJECT_NAME}
  ROS2_COMPONENT_CLASS "rr::DepthAIBaseRos2"
  SOURCES src/depthai_common.cpp
  ROS1_SOURCES src/depthai_base.cpp
  ROS2_SOURCES src/depthai_base_ros2.cpp
  ROS1_COMPONENT_SOURCES src/depthai_nodelet.cpp
  ROS1_EXE_SOURCES src/depthai_node.cpp
  ROS2_EXE_SOURCES src/depthai_node_ros2.cpp
  DIRECTORIES launch resources scripts urdf params
  INCLUDE_DIRS include
  SYSTEM_DEPS OpenCV Boost
  PKG_DEPS ${PKG_DEPS}
  TARGET_DEPS depthai-core)

if($ENV{ROS_VERSION} STREQUAL 1)
  install(FILES nodelet_plugins.xml DESTINATION ${depthai_ros_driver_PACKAGE_SHARE_DESTINATION})
endif()
