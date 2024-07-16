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



############
## EXPORT ##
############
cs_install()
cs_export()