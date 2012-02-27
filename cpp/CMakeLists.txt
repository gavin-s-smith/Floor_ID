cmake_minimum_required(VERSION 2.6)

PROJECT(SingleFrameExtraction)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# Default values
SET(NESTK_USE_FREENECT 0)
SET(NESTK_USE_OPENNI 1)
SET(NESTK_USE_EMBEDDED_OPENNI 0)
SET(NESTK_USE_PCL 1)
SET(NESTK_BUILD_SAMPLES 0)

ADD_SUBDIRECTORY(nestk)
## nestk/UseNestk.cmake.in defines variable to use nestk.
## It itself includes nestk/deps/cmake/UseEmbeddedNestkDeps.cmake that
## contains most of the cmake find packages commands.
INCLUDE("${nestk_BINARY_DIR}/UseEmbeddedNestk.cmake")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NESTK_EXTRA_CMAKE_CXX_FLAGS}" )
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

ADD_EXECUTABLE(SingleFrameExtraction SingleFrameExtraction.cpp)
TARGET_LINK_LIBRARIES(SingleFrameExtraction nestk boost_program_options)
