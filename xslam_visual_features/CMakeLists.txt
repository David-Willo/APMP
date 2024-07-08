cmake_minimum_required(VERSION 3.0)
project(xslam_visual_features)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fdiagnostics-color)
add_definitions(-Wextra -Wall -std=c++17)

# list(APPEND CMAKE_PREFIX_PATH "libtorch-shared-with-deps-1.13.1+cu117/libtorch/")
# find_package(Torch REQUIRED)

# find_package(CUDA  REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# # find_package(HDF5 REQUIRED)
# message(STATUS "#######")
# message(STATUS "torch include: ${TORCH_INCLUDE_DIRS}")
# message(STATUS "#######")
# message(STATUS ${TORCH_LIBRARIES})
	
# include_directories(
#   ${CUDA_INCLUDE_DIRS}
# 	# ${TORCH_INCLUDE_DIRS}
#     # ${PROJECT_SOURCE_DIR}/3rdparty/tensorrt/include
#     # ${PROJECT_SOURCE_DIR}/3rdparty/torch_tensorrt/include
# )

# 
# link_directories(${PROJECT_SOURCE_DIR}/3rdparty/tensorrt/lib)
# link_directories(${PROJECT_SOURCE_DIR}/3rdparty/torch_tensorrt/lib)
# file(GLOB TORCH_TRT_LIBS ${PROJECT_SOURCE_DIR}/3rdparty/torch_tensorrt/lib/*.so)
# file(GLOB TRT_LIBS ${PROJECT_SOURCE_DIR}/3rdparty/tensorrt/lib/*.so*)
# message(STATUS ${TORCH_TRT_LIBS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_lib
    src/superpoint.cc
    src/superpoint-jit.cc
    src/netvlad-jit.cc
    src/image_enhancement_worker.cc
    # src/distances.cc
    src/feature-factory.cc
)
# target_link_libraries(${PROJECT_NAME}_lib ${TORCH_LIBRARIES} ${TORCH_TRT_LIBS} ${TRT_LIBS})
target_link_libraries(${PROJECT_NAME}_lib)

#######
# APP #
#######
cs_add_executable(timing_models nodes/timing-models.cc)
target_link_libraries(timing_models ${PROJECT_NAME}_lib)

cs_add_executable(convert_models nodes/convert-models.cc)
target_link_libraries(convert_models ${PROJECT_NAME}_lib)


# cs_add_executable(test_minkindr_tf test/test_minkindr_tf.cpp)

#########
# SHARE #
#########

##########
# GTESTS #
##########

catkin_add_gtest(test_model_script
  tests/test-models.cc
)
target_link_libraries(test_model_script ${PROJECT_NAME}_lib)
xslam_import_test_models(test_model_script)

catkin_add_gtest(test_distances
  tests/test-distances.cc
)
target_link_libraries(test_distances ${PROJECT_NAME}_lib)
# xslam_import_test_maps(test_models)

############
## EXPORT ##
############
cs_install()
cs_export()