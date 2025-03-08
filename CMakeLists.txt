cmake_minimum_required(VERSION 3.0.2)
project(serial_stream)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(libserial)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES serial_stream
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LibSerial_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/serial_stream_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${SERIAL_LIBRARIES}
)
