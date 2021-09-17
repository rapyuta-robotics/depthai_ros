cmake_minimum_required(VERSION 3.5)

project(depthai_cmake)

set(depthai_cmake_VERSION 0.1.0)

include(GNUInstallDirs)

set(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_DATAROOTDIR}/depthai_cmake/cmake" CACHE PATH
  "Installation directory for CMake files")

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/depthai_cmake/depthai_cmakeConfig-version.cmake"
  VERSION "${depthai_cmake_VERSION}"
  COMPATIBILITY SameMajorVersion
  ARCH_INDEPENDENT)

configure_package_config_file(
  "depthai_cmakeConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/depthai_cmake/depthai_cmakeConfig.cmake"
  INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}"
  PATH_VARS "INSTALL_CMAKE_DIR")

configure_file(
  "depthai_cmake_macros.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/depthai_cmake/depthai_cmake_macros.cmake" @ONLY)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/depthai_cmake/depthai_cmakeConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/depthai_cmake/depthai_cmakeConfig-version.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/depthai_cmake/depthai_cmake_macros.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}")
