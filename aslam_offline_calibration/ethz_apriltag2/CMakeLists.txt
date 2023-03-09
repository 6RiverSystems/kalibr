cmake_minimum_required(VERSION 3.0.2)
project(ethz_apriltag2)
add_definitions(-std=c++14)

add_definitions(-fPIC -O3)

find_package(catkin REQUIRED COMPONENTS OpenCV cv_bridge)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosbag REQUIRED)
find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosconsole REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(image_transport REQUIRED)


catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
  rosbag
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)
include_directories(SYSTEM "/opt/ros/noetic/include")
include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS} ${Boost_LIBRARIES} ${OpenCV_LIBS})

#library
file(GLOB SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${rosbag_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

#demo
if(NOT APPLE)
  add_executable(apriltag_dem src/example/readTags.cpp)
  add_executable(apriltag_viz src/example/visualizer.cpp)
  target_link_libraries(apriltag_dem ${PROJECT_NAME} v4l2)
  target_link_libraries(apriltag_viz ${PROJECT_NAME} v4l2)
endif()

##################
## Installation ##
##################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
