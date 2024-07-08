cmake_minimum_required(VERSION 3.0)
project(xslam)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# set(CMAKE_CXX_STANDARD 17)
# add_definitions(-Wextra -Wall -std=c++17)

#############
# LIBRARIES #
#############

#######
# APP #
#######

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