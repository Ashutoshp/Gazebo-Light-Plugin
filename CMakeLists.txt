cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ignition-math2 QUIET REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${IGNITION-MATH_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${IGNITION-MATH_LIBRARY_DIRS})

#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#"-std=c++11")

add_library(control_light SHARED control_light.cc)
#target_link_libraries(velodyne_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})
target_link_libraries(control_light ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${roscpp_LIBRARIES} ${IGNITION-MATH_LIBRARIES})
#add_library(hello_world SHARED hello_world.cc)
#target_link_libraries(hello_world ${GAZEBO_LIBRARIES})
