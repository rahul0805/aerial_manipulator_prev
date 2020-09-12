cmake_minimum_required(VERSION 2.8.3)
project(serial_comm)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES serial_comm
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs 
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_library(serial_comm
  src/library/serial_comm.cpp
)

target_link_libraries(serial_comm ${catkin_LIBRARIES})
add_dependencies(serial_comm ${catkin_EXPORTED_TARGETS})

add_executable(serial_comm_node src/nodes/serial_comm_node.cpp)
add_dependencies(serial_comm_node $(catkin_EXPORTED_TARGETS))
target_link_libraries(serial_comm_node serial_comm ${catkin_LIBRARIES})


install(TARGETS serial_comm
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS serial_comm_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


