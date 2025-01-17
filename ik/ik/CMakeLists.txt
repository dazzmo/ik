find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(pinocchio REQUIRED)

# Collect source files
SET(${PROJECT_NAME}_SOURCES
common.cpp
data.cpp
dls.cpp
pik.cpp
problem.cpp
)

SET(${PROJECT_NAME}_LIBRARIES
Eigen3::Eigen
pinocchio::pinocchio
glog
)

include(GNUInstallDirs)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

message("CMAKE_CURRENT_LIST_DIR ${CMAKE_CURRENT_LIST_DIR}")

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>"
    # "$<INSTALL_INTERFACE:include>"
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wno-unused-parameter -pipe)

add_library(ik::ik ALIAS ${PROJECT_NAME})

## First of all, we need to install the library itself.
install(TARGETS ${PROJECT_NAME}
    EXPORT ik-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    # Perhaps use this, instead of installing include dir
    #PUBLIC_HEADER DESTINATION include
    INCLUDES DESTINATION include # We need this right?
    RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
