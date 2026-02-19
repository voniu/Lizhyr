cmake_minimum_required(VERSION 3.8)
project(lizhyr_restorer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(restorer_node
  src/restorer_node.cpp
  src/restorer_node_main.cpp
)

target_include_directories(restorer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(restorer_node
  rclcpp
  sensor_msgs
)

install(TARGETS restorer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
