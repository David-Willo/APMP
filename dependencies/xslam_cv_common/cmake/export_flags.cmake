include(${CMAKE_CURRENT_LIST_DIR}/detect_simd.cmake)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX14_EXTENSION_COMPILE_OPTION -std=c++17)
  set(CMAKE_CXX_STANDARD 17)
elseif(COMPILER_SUPPORTS_CXX11)
  add_definitions(--std=c++11)
else()
  message(FATAL_ERROR "Compiler does not support C++11 nor C++17!")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fext-numeric-literals")
endif()

set(ENABLE_TIMING FALSE CACHE BOOL "Set to TRUE to enable timing")
message(STATUS "Timers enabled? ${ENABLE_TIMING}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_TIMING=${ENABLE_TIMING}")

set(ENABLE_STATISTICS FALSE CACHE BOOL "Set to TRUE to enable statistics")
message(STATUS "Statistics enabled? ${ENABLE_STATISTICS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_STATISTICS=${ENABLE_STATISTICS}")