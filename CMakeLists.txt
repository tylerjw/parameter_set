cmake_minimum_required(VERSION 3.5)
project(parameter_set)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compile options set for the targets
set(PRIVATE_COMPILE_OPTIONS -Wall -Wextra -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
)

add_library(${PROJECT_NAME}
  src/parameter_set.cpp
  src/parameter_descriptor_builder.cpp
  src/validate_parameter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PRIVATE ${PRIVATE_COMPILE_OPTIONS})
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(example_node
  src/example_node/main.cpp
  src/example_node/robot_parameters.cpp
  src/example_node/robot_subsystem.cpp
)
ament_target_dependencies(example_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(example_node ${PROJECT_NAME})

# Install Libraries
install(
  TARGETS parameter_set
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install Binaries
install(
  TARGETS
    example_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
