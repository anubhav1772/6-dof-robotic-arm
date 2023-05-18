cmake_minimum_required(VERSION 3.0.2)
project(owr_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES owr_gazebo
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs xacro
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(state_publisher_node src/move_to_joint_sim.cpp)

## Add cmake target dependencies of the executable same as for the library above
add_dependencies(state_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(state_publisher_node ${catkin_LIBRARIES})

## https://stackoverflow.com/questions/30535933/eigen-gcc-5-class-stdbinder2nd-is-deprecated
## supress deprecation warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
