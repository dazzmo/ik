find_package(GTest REQUIRED)
find_package(gflags REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

enable_testing()

# Create list of files
SET(TEST_FILES
ik.cpp
dls.cpp
task.cpp
)

include(GoogleTest)

foreach(current_file IN LISTS TEST_FILES)
    get_filename_component(current_name ${current_file} NAME_WE)
    add_executable("${current_name}_unit_test" ${current_file})
    target_link_libraries("${current_name}_unit_test" ik GTest::gtest_main glog gflags)
endforeach()

# Install URDF to build folder
configure_file("${CMAKE_SOURCE_DIR}/test/ur5.urdf" "${CMAKE_BINARY_DIR}/ur5.urdf" COPYONLY)