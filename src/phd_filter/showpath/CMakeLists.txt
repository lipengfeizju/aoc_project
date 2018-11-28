cmake_minimum_required(VERSION 2.8.3)
project(showpath)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf

)
include_directories(include ${catkin_INCLUDE_DIRS})



## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()


add_executable(showpath src/showpath.cpp)
target_link_libraries(showpath ${catkin_LIBRARIES})
add_dependencies(showpath ${catkin_EXPORTED_TARGETS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ${catkin_EXPORTED_TARGETS})

add_executable(listen_show src/listen_show.cpp)
target_link_libraries(listen_show ${catkin_LIBRARIES})
add_dependencies(listen_show ${catkin_EXPORTED_TARGETS})

##N13geometry_msgs17TransformStamped_ISaIvEEE

