cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)
project(beex_2_xtf)

# Set C++ standard
add_compile_options(-std=c++17)

# Include BEEX_CMAKE.txt from bx_msgs
find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

# Find Catkin and required ROS packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    std_msgs
    nav_msgs
    bx_msgs
    sensor_msgs
    geometry_msgs
)

# Explicitly set the path for GeographicLib (adjust if installed elsewhere)
# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/usr/local")
find_package(GeographicLib REQUIRED)

# Define the catkin package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES beex_to_xtf_ros
    CATKIN_DEPENDS
        roscpp
        rosbag
        std_msgs
        nav_msgs
        bx_msgs
        sensor_msgs
        geometry_msgs
    DEPENDS GeographicLib
)

# Include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
)

# Define the executable
add_executable(beex_to_xtf_ros_node
    src/beex_to_xtf_ros.cpp
)

# Link libraries to the executable
target_link_libraries(beex_to_xtf_ros_node
    ${catkin_LIBRARIES}
    ${GeographicLib_LIBRARIES}
)

# Install targets
install(TARGETS beex_to_xtf_ros_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Optionally install config and launch files (if defined in BEEX_CMAKE.txt)
install_my_config_and_launch_files()