cmake_minimum_required(VERSION 3.0)
project(g2o_types)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# catkin_simple()
set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wextra -Wall -std=c++17)

# ############
# LIBRARIES #
# ############
cs_add_library(${PROJECT_NAME}_lib
    src/g2o_registration.cc
    src/g2o_visual_types.cc

    # src/topic-setting.cc

    # src/datastreamer.cc
    # src/datastset-kitti.cc
    # src/datastreamer-euroc.cc
    # src/datastreamer-rosbag.cc
)

# ######
# APP #
# ######
cs_add_executable(drummy_node node/drummy_node.cc)
target_link_libraries(drummy_node ${PROJECT_NAME}_lib)

# cs_add_executable(test_minkindr_tf test/test_minkindr_tf.cpp)

# ########
# SHARE #
# ########

# #########
# GTESTS #
# #########

# ###########
# # EXPORT ##
# ###########
cs_install()
cs_export()