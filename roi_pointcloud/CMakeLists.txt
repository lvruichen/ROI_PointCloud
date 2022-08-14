cmake_minimum_required(VERSION 3.0.2)
project(roi_pointcloud)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_roiFilter src/roiFilter.cpp)
target_link_libraries(${PROJECT_NAME}_roiFilter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
