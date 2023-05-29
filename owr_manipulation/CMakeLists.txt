cmake_minimum_required(VERSION 3.0.2)
project(owr_manipulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
  roscpp
  actionlib
  control_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  moveit_visual_tools
  rviz_visual_tools
  message_generation
  tf2_ros
  tf2
)
find_package(Boost REQUIRED)

catkin_package()

# include_directories(
#   SYSTEM
#   ${catkin_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
# )

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})


add_executable(joint_trajectory_control src/joint_trajectory_control.cpp)
add_dependencies(joint_trajectory_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_trajectory_control ${catkin_LIBRARIES})

## https://stackoverflow.com/questions/30535933/eigen-gcc-5-class-stdbinder2nd-is-deprecated
## supress deprecation warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

### INSTALL ###

#install(TARGETS joint_trajectory_control
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
