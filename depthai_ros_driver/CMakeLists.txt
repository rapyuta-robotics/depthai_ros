cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

set (CMAKE_CXX_STANDARD 17)

get_filename_component (DEPTHAI_CORE "${CMAKE_CURRENT_LIST_DIR}/../external/depthai-core" REALPATH)

include ("${DEPTHAI_CORE}/cmake/HunterGate.cmake")

set (CMAKE_POSITION_INDEPENDENT_CODE ON)
set (HUNTER_CONFIGURATION_TYPES "Release" CACHE STRING "Hunter dependencies list of build configurations")

HunterGate (
    URL "https://github.com/cpp-pm/hunter/archive/v0.23.258.tar.gz"
    SHA1 "062a19ab13ce8dffa9a882b6ce3e43bdabdf75d3"
    FILEPATH ${DEPTHAI_CORE}/cmake/Hunter/config.cmake # Add depthai-core config  (hunter limitation)
)

project (depthai_ros_driver VERSION 0.1.0 LANGUAGES CXX)

find_package (OpenCV REQUIRED)
find_package (Boost REQUIRED COMPONENTS system)

################################################
# Compile in ROS
################################################

if($ENV{ROS_VERSION} STREQUAL 1)

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

# Add depthai-core dependency
add_subdirectory (${DEPTHAI_CORE} depthai-core)

catkin_package (
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_nodelet
    CATKIN_DEPENDS ${PKG_DEPS}
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
add_library (${PROJECT_NAME}_lib src/depthai_base.cpp src/depthai_common.cpp)
target_link_libraries (${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    depthai-core
)
add_dependencies (${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library (${PROJECT_NAME}_nodelet src/depthai_nodelet.cpp)
target_link_libraries (${PROJECT_NAME}_nodelet
    ${PROJECT_NAME}_lib
)
add_executable (${PROJECT_NAME} src/depthai_node.cpp)
target_link_libraries (${PROJECT_NAME}
    ${PROJECT_NAME}_lib
)

#############
## Install ##
#############

install (TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_nodelet ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Not installing the headers because this is not a library package
install (DIRECTORY launch resources scripts urdf params
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
)

install (FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
RETURN()
else()
  message(VERBOSE "ROS version is $ENV{ROS_VERSION}, skipping ROS1 ${PROJECT_NAME}")
endif()

################################################
# Compile in ROS 2
################################################
if($ENV{ROS_VERSION} STREQUAL 2)
    find_package(ament_cmake REQUIRED)
    set (PKG_DEPS
        camera_info_manager
        cv_bridge
        depthai_ros_msgs
        image_transport
        rclcpp
        sensor_msgs
        std_msgs
        rclcpp_components
    )
    foreach(pkg ${PKG_DEPS})
        find_package(${pkg} REQUIRED)
    endforeach()

    add_subdirectory (${DEPTHAI_CORE} depthai-core)

    add_definitions(-DUSE_ROS2)   # <-------- Set to use ros2 macro, for #ifdef
    add_library (${PROJECT_NAME}_lib SHARED
        src/depthai_common.cpp
        src/depthai_base_ros2.cpp
    )
    target_link_libraries (${PROJECT_NAME}_lib PUBLIC
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES}
        depthai-core
    )
    ament_target_dependencies(${PROJECT_NAME}_lib 
        PUBLIC ${PKG_DEPS}
    )

    # Generate ros2 component
    rclcpp_components_register_nodes(${PROJECT_NAME}_lib  "rr::DepthAIBaseRos2")
    
    include_directories (include)
    add_executable (${PROJECT_NAME}_node src/depthai_node_ros2.cpp)
    target_link_libraries(${PROJECT_NAME}_node PUBLIC ${PROJECT_NAME}_lib)

    ##########################
    # Install ros2 component
    ##########################
    install(TARGETS
        ${PROJECT_NAME}_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

    install(DIRECTORY
        launch/
        params
        resources
        urdf
        DESTINATION share/${PROJECT_NAME}
    )

    install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

    ament_package()
else()
    message(VERBOSE "ROS version is $ENV{ROS_VERSION}, skipping this ROS2 ${PROJECT_NAME}")
endif()
