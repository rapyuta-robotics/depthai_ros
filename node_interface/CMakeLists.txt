cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

if (POLICY CMP0076)
  cmake_policy (SET CMP0076 NEW)
endif ()

project (node_interface VERSION 0.1.0 LANGUAGES CXX)

## Find catkin macros and libraries
find_package (catkin REQUIRED
  image_transport
  roscpp
)

###################################
## catkin specific configuration ##
###################################
catkin_package (
  INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########
include_directories (SYSTEM ${catkin_INCLUDE_DIRS})
include_directories (include)

set (HEADERS
  include/${PROJECT_NAME}/introspection.hpp
  include/${PROJECT_NAME}/ros1_node_interface.hpp
  include/${PROJECT_NAME}/ros1_reactive_node_interface.hpp
)
add_library (${PROJECT_NAME} INTERFACE)
target_sources (${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${HEADERS}>)
target_include_directories (${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories (${PROJECT_NAME} SYSTEM INTERFACE $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>)
target_compile_features (${PROJECT_NAME} INTERFACE
  cxx_constexpr
  cxx_generic_lambdas
  cxx_nonstatic_member_init
  cxx_nullptr
  cxx_right_angle_brackets
  cxx_uniform_initialization
  cxx_variable_templates
)
#############
## Install ##
#############

install (TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install (DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
