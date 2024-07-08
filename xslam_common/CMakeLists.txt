cmake_minimum_required(VERSION 3.0)
project(xslam_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wextra -Wall -std=c++17)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_lib 
    src/math.cc
    src/frame.cc
    src/timing.cc
    src/cv_helper.cc
)

#######
# APP #
#######
# cs_add_executable(test_sensor_manager test/test_sensor_manager.cpp)
# target_link_libraries(test_sensor_manager c++fs)

# cs_add_executable(test_minkindr_tf test/test_minkindr_tf.cpp)

#########
# SHARE #
#########

##########
# GTESTS #
##########

############
## EXPORT ##
############
cs_install()
cs_export()