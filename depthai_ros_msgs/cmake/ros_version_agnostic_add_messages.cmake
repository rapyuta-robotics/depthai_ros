# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

macro(add_ros_messages)
  cmake_parse_arguments(_ARG
    ""
    "PROJECT_NAME"
    "MESSAGES;SERVICES;PKG_DEPS"
    ${ARGN})
  if(_ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "add_ros_messages() called with unused arguments: '"
      "${_ARG_UNPARSED_ARGUMENTS}'")
  endif()

  if($ENV{ROS_VERSION} STREQUAL 1)
    message(STATUS "Compiling ${PROJECT_NAME} as ROS 1")

    find_package(catkin REQUIRED COMPONENTS message_generation ${_ARG_PKG_DEPS})

    if(NOT "${_ARG_MESSAGES}" STREQUAL "")
      add_message_files(FILES ${_ARG_MESSAGES})
    endif()

    if(NOT "${_ARG_SERVICES}" STREQUAL "")
      add_service_files(FILES ${_ARG_SERVICES})
    endif()

    generate_messages(DEPENDENCIES ${_ARG_PKG_DEPS})

    catkin_package(CATKIN_DEPENDS message_runtime ${_ARG_PKG_DEPS})
  else()
    message(STATUS "Compiling ${PROJECT_NAME} as ROS 2")

    find_package(builtin_interfaces REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    foreach(pkg_dep ${_ARG_PKG_DEPS})
      find_package(${pkg_dep} REQUIRED)
    endforeach()

    set(message_files "")
    foreach(message ${_ARG_MESSAGES})
      set(message_files "msg/${message};${message_files}")
    endforeach()
    set(service_files "")
    foreach(service ${_ARG_SERVICES})
      set(service_files "srv/${service};${service_files}")
    endforeach()

    rosidl_generate_interfaces(${_ARG_PROJECT_NAME} ${service_files} ${message_files}
      DEPENDENCIES builtin_interfaces ${_ARG_PKG_DEPS}
      ADD_LINTER_TESTS
    )

    ament_export_dependencies(rosidl_default_runtime)
    ament_package()
  endif()
endmacro()
