cmake_minimum_required(VERSION 2.8.3)
project(image_filter)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  sensor_msgs
  rosconsole
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


 add_executable(image_filter_node src/image_filter_node.cpp)

 target_link_libraries(image_filter_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

