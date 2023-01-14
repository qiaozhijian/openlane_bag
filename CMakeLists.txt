cmake_minimum_required(VERSION 3.5)
project(openlane_bag)

set(DEFAULT_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")


include(cmake/packages.cmake)

#catkin_install_python(PROGRAMS
#        openlane_rosbag.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )