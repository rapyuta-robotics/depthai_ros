cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

set (CMAKE_CXX_STANDARD 17)

get_filename_component (DEPTHAI_CORE "${CMAKE_CURRENT_LIST_DIR}/../external/depthai-core" REALPATH)
get_filename_component (DEPTHAI_SHARED "${CMAKE_CURRENT_LIST_DIR}/../external/depthai-core/shared/depthai-shared" REALPATH)

include ("${DEPTHAI_CORE}/cmake/HunterGate.cmake")

set (CMAKE_POSITION_INDEPENDENT_CODE ON)
set (HUNTER_CONFIGURATION_TYPES "Release" CACHE STRING "Hunter dependencies list of build configurations")
set (MSGPACK_CXX17 ON)

HunterGate (
    URL "https://github.com/cpp-pm/hunter/archive/v0.23.304.tar.gz"
    SHA1 "cae9026e69d7d8333897663688a11f4232fb8826"
    FILEPATH ${DEPTHAI_CORE}/cmake/Hunter/config.cmake # Add depthai-core config  (hunter limitation)
)

cmake_policy (SET CMP0048 NEW)
project (depthai_ros_driver VERSION 0.1.0 LANGUAGES C CXX)  # C needed for dependencies

set (PKG_DEPS
    camera_info_manager
    cv_bridge
    depthai_common_msgs
    depthai_datatype_msgs
    depthai_ros_msgs
    image_transport
    message_generation
    nodelet
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    topic_tools
    node_interface
)

find_package (catkin REQUIRED COMPONENTS ${PKG_DEPS})

find_package (OpenCV REQUIRED)
find_package (Boost REQUIRED COMPONENTS system)

hunter_add_package(range-v3)
find_package(range-v3 REQUIRED)

# Add depthai-core dependency
add_subdirectory (${DEPTHAI_CORE} depthai-core)

# needed for conda-forge
find_package(PkgConfig)
pkg_check_modules(libudev libudev)

catkin_package (
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_nodelet
    CATKIN_DEPENDS camera_info_manager cv_bridge
    depthai_common_msgs depthai_datatype_msgs depthai_ros_msgs
    image_transport nodelet roscpp sensor_msgs node_interface
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

add_library (${PROJECT_NAME}_pipelines
    src/pipelines/preview_pipeline.cpp
    src/pipelines/stereo_pipeline.cpp
    src/pipelines/mobilenet_ssd_pipeline.cpp
    src/pipelines/stereo_with_mobilened_ssd_pipeline.cpp
    src/pipelines/mono_camera_control_pipeline.cpp
)
target_link_libraries (${PROJECT_NAME}_pipelines PRIVATE
    ${catkin_LIBRARIES}
    depthai-core
)

# add_library (${PROJECT_NAME}_nodelet src/depthai_nodelet.cpp)
# target_link_libraries (${PROJECT_NAME}_nodelet
#     ${PROJECT_NAME}_lib
# )
# add_executable (${PROJECT_NAME} src/depthai_node.cpp)
# target_link_libraries (${PROJECT_NAME}
#     ${PROJECT_NAME}_lib
# )

add_library(depthai INTERFACE)
target_link_libraries(depthai INTERFACE depthai-core ${libudev_LIBRARIES})
target_include_directories(depthai SYSTEM INTERFACE
    ${libudev_INCLUDE_DIRS}
)

add_library(gen2_ros_conversion src/conversion.cpp)
target_link_libraries(gen2_ros_conversion ${catkin_LIBRARIES} depthai ${OpenCV_LIBS})

add_library(gen2_ros_pipeline_loader src/pipeline_loader.cpp)
target_link_libraries(gen2_ros_pipeline_loader ${catkin_LIBRARIES} depthai-core)

add_library(gen2_ros_queue src/dai_utils.cpp)
target_link_libraries(gen2_ros_queue ${catkin_LIBRARIES} depthai range-v3)

add_executable(gen2_ros_driver src/depthai_loader.cpp)
target_link_libraries(gen2_ros_driver gen2_ros_queue gen2_ros_pipeline_loader gen2_ros_conversion)

add_library(common_stampers src/common_stampers.cpp)
target_link_libraries(common_stampers ${catkin_LIBRARIES} depthai-core)

add_library(common_stamper_nodelets src/common_stamper_nodelets.cpp)
target_link_libraries(common_stamper_nodelets common_stampers)

add_executable(img_detections_stamper_node src/img_detections_stamper_node.cpp)
target_link_libraries(img_detections_stamper_node common_stampers)

catkin_add_gtest(${PROJECT_NAME}_depthai_api test/depthai_api.cpp)
catkin_add_gtest(${PROJECT_NAME}_dai_utils test/dai_utils.cpp)
catkin_add_gtest(${PROJECT_NAME}_conversion test/conversion.cpp)
if(TARGET ${PROJECT_NAME}_depthai_api)
    target_link_libraries(${PROJECT_NAME}_depthai_api gen2_ros_queue gtest_main)
    target_compile_options(${PROJECT_NAME}_depthai_api PRIVATE "-g")
endif()
if(TARGET ${PROJECT_NAME}_dai_utils)
    target_link_libraries(${PROJECT_NAME}_dai_utils gen2_ros_queue gtest_main)
    target_compile_options(${PROJECT_NAME}_dai_utils PRIVATE "-g")
endif()
if(TARGET ${PROJECT_NAME}_conversion)
    target_link_libraries(${PROJECT_NAME}_conversion gen2_ros_conversion gtest_main)
    target_compile_options(${PROJECT_NAME}_conversion PRIVATE "-g")
endif()

#############
## Install ##
#############

install (TARGETS gen2_ros_conversion gen2_ros_pipeline_loader gen2_ros_queue 
  gen2_ros_driver common_stampers common_stamper_nodelets
  img_detections_stamper_node ${PROJECT_NAME}_pipelines
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Not installing the headers because this is not a library package
install (DIRECTORY launch resources scripts urdf params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
install (FILES
  nodelet_plugins.xml pipeline_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
