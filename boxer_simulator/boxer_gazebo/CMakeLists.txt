cmake_minimum_required(VERSION 2.8.3)
project(boxer_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS gazebo_ros roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY launch worlds meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/boxer_base_sim
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
