CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(ffld_ros)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  ffld_nx
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
  cmake_modules
  image_transport
  image_geometry
  camera_info_manager
)

# Generate services in the 'srv' folder
add_service_files(
 FILES
 DetectObject.srv
 SetDpmParams.srv
)

# search for all msg files
#FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
#    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

# notify catkin to look at the previously found msg files
add_message_files(
  FILES
  ObjectDetection.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs  # Or other packages containing msgs
 sensor_msgs
)


## Add plain cmake packages
FIND_PACKAGE(Eigen 3 REQUIRED)
find_package(OpenCV REQUIRED)

# Describe catkin Project
catkin_package(
  DEPENDS OpenCV Eigen #Sophus Boost
  CATKIN_DEPENDS roscpp message_runtime std_msgs sensor_msgs ffld_nx cv_bridge
  INCLUDE_DIRS include
  LIBRARIES
)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${EIGEN_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Set link libraries
LIST(APPEND LINK_LIBS
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

# Create Executables
add_executable(detect_object_server src/detect_object_server.cpp)
target_link_libraries(detect_object_server ${LINK_LIBS})
add_dependencies(detect_object_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(detect_object_node src/detect_object_node.cpp)
target_link_libraries(detect_object_node ${LINK_LIBS})
add_dependencies(detect_object_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(detect_object_client src/detect_object_client.cpp)
target_link_libraries(detect_object_client ${LINK_LIBS})
add_dependencies(detect_object_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(dpm_detect_service src/dpm_detect_service.cpp)
target_link_libraries(dpm_detect_service ${LINK_LIBS})
add_dependencies(dpm_detect_service ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(fla_dpm_detector src/fla_dpm_detector.cpp src/dpm_detect_client.cpp include/ffld_ros/fla_dpm_detector.h)
#target_link_libraries(fla_dpm_detector ${LINK_LIBS})
#add_dependencies(fla_dpm_detector ${${PROJECT_NAME}_EXPORTED_TARGETS})

# not strictly needed
#add_executable(object_detector_launcher_node src/object_detector_launcher_node.cpp)
#target_link_libraries(object_detector_launcher_node ${LINK_LIBS})
#add_dependencies(object_detector_launcher_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(object_detection_visualizer src/object_detection_visualizer.cpp)
#target_link_libraries(object_detection_visualizer ${LINK_LIBS})
#add_dependencies(object_detection_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(pub_image_once_node src/pub_image_once_node.cpp)
#target_link_libraries(pub_image_once_node ${LINK_LIBS})
#add_dependencies(pub_image_once_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
