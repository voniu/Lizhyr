cmake_minimum_required(VERSION 3.8)
project(lizhyr_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(processor_node
  src/processor_node.cpp
  src/processor_node_main.cpp
)

target_include_directories(processor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(processor_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

if(PCL_DEFINITIONS)
  target_compile_definitions(processor_node PRIVATE ${PCL_DEFINITIONS})
endif()

target_link_libraries(processor_node ${PCL_LIBRARIES})

install(TARGETS processor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
