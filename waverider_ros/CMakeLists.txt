cmake_minimum_required(VERSION 3.0.2)
project(waverider_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Libraries
cs_add_library(${PROJECT_NAME}
    src/policy_visuals.cc
    src/ros_server.cc)
set_wavemap_target_properties(${PROJECT_NAME})

# Binaries
cs_add_executable(ros_server app/ros_server.cc)
target_link_libraries(ros_server ${PROJECT_NAME})
set_wavemap_target_properties(ros_server)

# Export
cs_install()
cs_export()
