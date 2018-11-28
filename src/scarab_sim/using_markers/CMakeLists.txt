cmake_minimum_required(VERSION 2.8.3)
project(using_markers)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wno-deprecated-declarations")



FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp nodelet)
  catkin_package(
      DEPENDS # JPEG LibXml2 Eigen Eigen3 
      CATKIN_DEPENDS roscpp
      INCLUDE_DIRS
      LIBRARIES marker_visualization
  )

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


################################################################################
add_library(marker_visualization SHARED
  src/vel_visualization.cpp
  src/pose_visualization.cpp
  src/line_visualization.cpp
  #src/cmd_visualization.cpp
  src/covar_visualization.cpp)
target_link_libraries(marker_visualization ${catkin_LIBRARIES})

add_executable(environment_visualization src/mesh_visualization.cpp)
target_link_libraries(environment_visualization ${catkin_LIBRARIES})

#add_executable(quad_visualization src/quad_visualization.cpp)
#add_executable(map_vis src/map_vis.cpp)
#add_executable(traj_vis src/traj_vis.cpp)






#cmake_minimum_required(VERSION 2.8.3)
#project(using_markers)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wno-deprecated-declarations")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../trajectory_solver/cmake")
#find_package(GUROBI REQUIRED)
#find_package(catkin_simple REQUIRED)
#find_package(cmake_modules)
#find_package(Eigen REQUIRED)
#include_directories(${EIGEN_INCLUDE_DIRS})
#	 ${GUROBI_INCLUDE_DIRS})

#catkin_simple()


#cs_add_library(marker_visualization
#  src/vel_visualization.cpp
#  src/pose_visualization.cpp
#  src/line_visualization.cpp
#  src/cmd_visualization.cpp
#  src/covar_visualization.cpp)

#cs_add_executable(quad_visualization
#  src/quad_visualization.cpp)

#cs_add_executable(environment_visualization
#  src/mesh_visualization.cpp)

#cs_add_executable(map_vis src/map_vis.cpp)
#cs_add_executable(traj_vis src/traj_vis.cpp)

#cs_install()

#cs_export()


