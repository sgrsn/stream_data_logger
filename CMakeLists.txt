cmake_minimum_required(VERSION 3.8)
project(serial_stream)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_path(LIBSERIAL_INCLUDE_DIR libserial/SerialStream.h)
find_library(LIBSERIAL_LIBRARY NAMES serial)

if(NOT LIBSERIAL_INCLUDE_DIR OR NOT LIBSERIAL_LIBRARY)
  message(FATAL_ERROR "libserial headers or library not found")
endif()

add_executable(${PROJECT_NAME}_node src/serial_stream_node.cpp)

target_include_directories(${PROJECT_NAME}_node PRIVATE ${LIBSERIAL_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_node ${LIBSERIAL_LIBRARY})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
