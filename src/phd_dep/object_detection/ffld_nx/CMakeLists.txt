#                          Implementation of the papers
#
#                "Exact Acceleration of Linear Object Detectors"
#               12th European Conference on Computer Vision, 2012.
#
#             "Deformable Part Models with Individual Part Scaling"
#                  24th British Machine Vision Conference, 2013.
#
#      Copyright (c) 2013 Idiap Research Institute, <http://www.idiap.ch/>
#              Written by Charles Dubout <charles.dubout@idiap.ch>


################################################################################
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(ffld_nx)

# user build settings
OPTION(USE_ROS "Set OFF if you want to build this package without Catkin" ON)
OPTION(FFLD_HOGPYRAMID_DOUBLE "Use doubles instead of floats (slower, uses twice more memory, and the increase in precision is not necessarily useful)." OFF)
OPTION(FFLD_HOGPYRAMID_EXTRA_FEATURES "Use extra features (LBP + color) in addition to HOG." OFF)
OPTION(FFLD_MODEL_3D "Allow parts to also deform across scales." OFF)
OPTION(FFLD_MIXTURE_STANDARD_CONVOLUTION "Use standard convolutions instead of the optimized Fourier ones." OFF)
################################################################################

# Select a default build configuration if none was chosen
#SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()

# Set definitions
IF(FFLD_HOGPYRAMID_DOUBLE)
  MESSAGE(STATUS "Use doubles instead of floats.")
  ADD_DEFINITIONS(-DFFLD_HOGPYRAMID_DOUBLE)
ENDIF()
IF(FFLD_HOGPYRAMID_EXTRA_FEATURES)
  MESSAGE(STATUS "Use extra features (LBP + color) in addition to HOG.")
  ADD_DEFINITIONS(-DFFLD_HOGPYRAMID_EXTRA_FEATURES)
ENDIF()
IF(FFLD_MODEL_3D)
  MESSAGE(STATUS "Allow parts to also deform across scales.")
  ADD_DEFINITIONS(-DFFLD_MODEL_3D)
ENDIF()
IF(FFLD_MIXTURE_STANDARD_CONVOLUTION)
  MESSAGE(STATUS "Use standard convolutions instead of the optimized Fourier ones.")
  ADD_DEFINITIONS(-DFFLD_MIXTURE_STANDARD_CONVOLUTION)
ENDIF()
IF(USE_ROS)
  MESSAGE(STATUS "Using catkin to compile.")
ENDIF()

IF(USE_ROS)
  # roslib cmake_modules
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp cmake_modules)
  catkin_package(
      DEPENDS JPEG LibXml2 # Eigen Eigen3
      CATKIN_DEPENDS roscpp
      INCLUDE_DIRS include
      LIBRARIES ffld2
  )
ELSE()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# Add plain cmake packages
FIND_PACKAGE(Eigen 3 REQUIRED)
FIND_PACKAGE(JPEG REQUIRED)
FIND_PACKAGE(LibXml2 REQUIRED)
FIND_PACKAGE(Boost 1.40 COMPONENTS filesystem system REQUIRED )
ADD_DEFINITIONS(${LIBXML2_DEFINITIONS})

# There are no CMake FFTW package, so find it ourselves
FIND_PATH(FFTW3_INCLUDE_DIR fftw3.h)
IF(FFLD_HOGPYRAMID_DOUBLE)
  FIND_LIBRARY(FFTW3_LIBRARIES fftw3)
ELSE()
  FIND_LIBRARY(FFTW3_LIBRARIES fftw3f)
ENDIF()
IF(NOT FFTW3_INCLUDE_DIR OR NOT FFTW3_LIBRARIES)
  MESSAGE(FATAL_ERROR "Could not find fftw3.")
ENDIF()

# Not required, but stronlgy recommended on multi-core systems
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_C_FLAGS}")
ENDIF()

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${EIGEN_INCLUDE_DIR}
  ${JPEG_INCLUDE_DIR}
  ${LIBXML2_INCLUDE_DIR}
  ${FFTW3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

# Set link libraries
LIST(APPEND LINK_LIBS
  ${JPEG_LIBRARIES}
  ${LIBXML2_LIBRARIES}
  ${FFTW3_LIBRARIES}
)

IF(USE_ROS)
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ENDIF()

# Set sourcefiles
LIST(APPEND SOURCEFILES
  src/HOGPyramid.cpp
  src/JPEGImage.cpp
  src/LBFGS.cpp
  src/Mixture.cpp
  src/Model.cpp
  src/Object.cpp
  src/Patchwork.cpp
  src/Rectangle.cpp
  src/Scene.cpp)

LIST(APPEND HEADERFILES
  include/HOGPyramid.h
  include/Intersector.h
  include/JPEGImage.h
  include/LBFGS.h
  include/Mixture.h
  include/Model.h
  include/Object.h
  include/Patchwork.h
  include/Rectangle.h
  include/Scene.h
  include/SimpleOpt.h
  include/FFLDDetector.hpp)

################################################################################
# Create ffld2 library
#ADD_LIBRARY(ffld2 STATIC ${HEADERFILES} ${SOURCEFILES})
ADD_LIBRARY(ffld2 SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(ffld2 ${LINK_LIBS})


################################################################################
# TESTS
#ADD_EXECUTABLE(train src/train.cpp)
#target_link_libraries(train ffld2)
#ADD_EXECUTABLE(detect src/test.cpp)
#target_link_libraries(detect ffld2)
ADD_EXECUTABLE(train_FFLDDetector src/train_FFLDDetector.cpp)
target_link_libraries(train_FFLDDetector ${Boost_LIBRARIES} ffld2)

ADD_EXECUTABLE(test_FFLDDetector src/test_FFLDDetector.cpp)
target_link_libraries(test_FFLDDetector ffld2)
