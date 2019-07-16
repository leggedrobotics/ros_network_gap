cmake_minimum_required(VERSION 2.8.3)
project(ros_network_gap)

add_definitions(-std=c++11 -Wall)

set(PACKAGE_DEPENDENCIES
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

install(DIRECTORY launch bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)