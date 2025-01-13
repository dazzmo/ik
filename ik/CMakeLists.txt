cmake_minimum_required(VERSION 3.20)
project(ik VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3 -fPIC -O3")

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

IF(APPLE)
    # Fix linking on 10.14+. See https://stackoverflow.com/questions/54068035
    LINK_DIRECTORIES(/usr/local/lib)
ENDIF()

# Add ik library
add_subdirectory(ik)

option(BUILD_TESTS "Build ik with tests" OFF)
if(BUILD_TESTS)
  add_subdirectory(test)
endif(BUILD_TESTS)

## We install the export that we defined above
## Export the targets to a script
## This will install the import script ikTargets.cmake
## When findpackaged by other scripts, it will load the targets defined
## in the export ik-export.
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/ikConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# Create ikConfig.cmake with extra info from ikConfig.cmake.in
# This file is necessary to find_package the library ikConfig.
set(INSTALL_CONFIGDIR lib/cmake/ik)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/ikConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/ikConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

# Install config and export files
install(EXPORT ik-export
  FILE ikTargets.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
  NAMESPACE ik::
)

## Install the config and configversion
install(FILES
${CMAKE_CURRENT_BINARY_DIR}/ikConfig.cmake
${CMAKE_CURRENT_BINARY_DIR}/ikConfigVersion.cmake
# ${CMAKE_CURRENT_LIST_DIR}/cmake/FindGflags.cmake
# ${CMAKE_CURRENT_LIST_DIR}/cmake/FindGlog.cmake
DESTINATION ${INSTALL_CONFIGDIR}
)
