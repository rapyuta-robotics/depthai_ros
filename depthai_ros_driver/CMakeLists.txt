cmake_minimum_required (VERSION 2.8.3)

set (CMAKE_CXX_STANDARD 14)

get_filename_component (DEPTHAI_CORE "${CMAKE_CURRENT_LIST_DIR}/../external/depthai-core" REALPATH)

include ("${DEPTHAI_CORE}/cmake/HunterGate.cmake")

set (CMAKE_POSITION_INDEPENDENT_CODE ON)
set (HUNTER_CONFIGURATION_TYPES "Release" CACHE STRING "Hunter dependencies list of build configurations")

HunterGate (
    URL "https://github.com/cpp-pm/hunter/archive/v0.23.258.tar.gz"
    SHA1 "062a19ab13ce8dffa9a882b6ce3e43bdabdf75d3"
    FILEPATH ${DEPTHAI_CORE}/cmake/Hunter/config.cmake # Add depthai-core config  (hunter limitation)
)

cmake_policy (SET CMP0048 NEW)
project (depthai_ros_driver VERSION 0.1.0 LANGUAGES C CXX)

set (PKG_DEPS
    camera_info_manager
    cv_bridge
    depthai_ros_msgs
    image_transport
    message_generation
    nodelet
    roscpp
    sensor_msgs
    std_msgs
    node_interface
)

find_package (catkin REQUIRED COMPONENTS ${PKG_DEPS})

find_package (OpenCV REQUIRED)
find_package (Boost REQUIRED)

# Add depthai-core dependency
add_subdirectory (${DEPTHAI_CORE} depthai-core)

hunter_add_package(range-v3)
find_package(range-v3 REQUIRED)

catkin_package (
    INCLUDE_DIRS include
    # LIBRARIES ${PACKAGE_NAME}_nodelet
    CATKIN_DEPENDS camera_info_manager cv_bridge image_transport nodelet roscpp sensor_msgs node_interface
    DEPENDS Boost
)

###########
## Build ##
###########

include_directories (include)

include_directories (SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# internal library, not to be exported
add_library (${PROJECT_NAME}_lib src/depthai_base.cpp)
target_link_libraries (${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    depthai-core
)
add_dependencies (${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library (${PROJECT_NAME}_pipeline
    src/pipeline.cpp
    src/pipelines/preview_pipeline.cpp
    src/pipelines/stereo_pipeline.cpp
    src/pipelines/mobilenet_ssd_pipeline.cpp
)
target_link_libraries (${PROJECT_NAME}_pipeline
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    depthai-core
)
target_link_libraries (${PROJECT_NAME}_pipeline
    depthai-core
)
# add_dependencies (${PROJECT_NAME}_pipeline ${catkin_EXPORTED_TARGETS})

# add_library (${PROJECT_NAME}_nodelet src/depthai_nodelet.cpp)
# target_link_libraries (${PROJECT_NAME}_nodelet
#     ${PROJECT_NAME}_lib
# )
add_executable (${PROJECT_NAME} src/depthai_node.cpp)
target_link_libraries (${PROJECT_NAME}
    ${PROJECT_NAME}_lib
)

#############
## Install ##
#############

# install (TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_nodelet ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Not installing the headers because this is not a library package
install (DIRECTORY launch resources scripts urdf params
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
)

# install (FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
