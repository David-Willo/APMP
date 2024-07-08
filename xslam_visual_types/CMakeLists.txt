cmake_minimum_required(VERSION 3.0)
project(xslam_visual_types)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fdiagnostics-color)
add_definitions(-Wextra -Wall -std=c++17)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_lib 
    src/mappoint.cc
    src/keypoint-grid.cc
    src/visual-frame.cc
    src/visual-map.cc
    src/tile.cc
)

target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${PCL_INCLUDE_DIRS})

#######
# APP #
#######
# cs_add_executable(test_dataset test/test_dataset.cpp)
# target_link_libraries(test_dataset ${PROJECT_NAME}_lib)

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