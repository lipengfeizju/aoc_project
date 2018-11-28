cmake_minimum_required(VERSION 2.8.3)
project(truncate_scan)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
catkin_package(
      DEPENDS # JPEG LibXml2 Eigen Eigen3 
      CATKIN_DEPENDS roscpp sensor_msgs
      INCLUDE_DIRS #include
      LIBRARIES #${PROJECT_NAME}
  )

include_directories(${catkin_INCLUDE_DIRS})

add_executable(truncate_scan src/truncate_scan.cpp)
target_link_libraries(truncate_scan ${catkin_LIBRARIES})


