cmake_minimum_required(VERSION 3.0.2)
project(waverider)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Libraries
cs_add_library(${PROJECT_NAME}
    src/goal_policy.cc
    src/goal_policy_tuning.cc
    src/obstacle_filter.cc
    src/parallelized_policy.cc
    src/obstacle_policy_tuning.cc
    src/waverider_policy.cc
    src/yaw_policy.cc
    src/yaw_policy_tuning.cc)
set_wavemap_target_properties(${PROJECT_NAME})

# Export
cs_install()
cs_export()
