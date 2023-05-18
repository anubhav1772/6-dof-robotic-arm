cmake_minimum_required(VERSION 3.0.2)
project(owr_description)

FIND_PACKAGE(catkin REQUIRED)
CATKIN_PACKAGE()

INCLUDE_DIRECTORIES()

## https://stackoverflow.com/questions/30535933/eigen-gcc-5-class-stdbinder2nd-is-deprecated
## supress deprecation warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

#############
## Install ##
#############

INSTALL(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
INSTALL(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
