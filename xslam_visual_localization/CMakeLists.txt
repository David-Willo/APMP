cmake_minimum_required(VERSION 3.0)
project(xslam_visual_localization)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fdiagnostics-color)
add_definitions(-Wextra -Wall -std=c++17)

add_subdirectory(highfive)

# catkin simple cannnot auto link include dir for non catkin pkgs
include_directories(${PCL_INCLUDE_DIRS})

# find_package(HDF5 REQUIRED)
# message(status ${XSLAM_TEST_DATA_DIR})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_lib 
    src/map-io.cc
    src/matching.cc
    src/optimization.cc
    src/relocalization.cc
    src/tracking.cc
    src/visual-localization.cc
    src/geo_observer.cc
    src/visualization.cc
    src/debug_visualize.cc
    src/wavelet/CWavelet.cc
)
target_link_libraries(${PROJECT_NAME}_lib HighFive faiss lapack ${PCL_LIBRARIES})

#######
# APP #
#######
cs_add_executable(visual_localization_node nodes/visual-localization-node.cc)
target_link_libraries(visual_localization_node ${PROJECT_NAME}_lib)

cs_add_executable(evaluate_map nodes/evaluate_map.cc)
target_link_libraries(evaluate_map ${PROJECT_NAME}_lib)

# cs_add_executable(test_minkindr_tf test/test_minkindr_tf.cpp)

#########
# SHARE #
#########

##########
# GTESTS #
##########

catkin_add_gtest(test_map_io_sample
  tests/test-map-loading.cc
)
target_link_libraries(test_map_io_sample ${PROJECT_NAME}_lib)
xslam_import_test_maps(test_map_io_sample)

catkin_add_gtest(test_feature_extraction
  tests/test-feature-extraction.cc
)
target_link_libraries(test_feature_extraction ${PROJECT_NAME}_lib)
xslam_import_test_maps(test_feature_extraction)
xslam_import_test_models(test_feature_extraction)

catkin_add_gtest(test_relocalization
  tests/test-relocalization.cc
)
target_link_libraries(test_relocalization ${PROJECT_NAME}_lib)
xslam_import_test_maps(test_relocalization)
xslam_import_test_models(test_relocalization)

############
## EXPORT ##
############
cs_install()
cs_export()