cmake_minimum_required(VERSION 3.8)
project(tesi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(dynamicEDT3D REQUIRED)

# Specify the path and library for dynamicEDT3D
set(dynamicEDT3D_DIR "/opt/ros/humble/share/dynamicEDT3D")
set(DYNAMICEDT3D_LIBRARIES "/opt/ros/humble/lib/libdynamicedt3d.so")

# Include directories and link libraries
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
include_directories(${OMPL_INCLUDE_DIRS})

link_libraries(${DYNAMICEDT3D_LIBRARIES})
link_libraries(${OMPL_LIBRARIES})

# Define the executable target correctly
# add_executable(examples examples/src/main.cpp)
# ament_target_dependencies(examples rclcpp std_msgs ompl geometry_msgs)

add_executable(octo_publisher tesi/src/octo_publisher.cpp)
ament_target_dependencies(octo_publisher rclcpp std_msgs octomap_msgs octomap)  

add_executable(octo_planner tesi/src/octo_planner.cpp)
ament_target_dependencies(octo_planner rclcpp std_msgs octomap_msgs octomap ompl geometry_msgs dynamicEDT3D)

add_executable(octo_benchmark tesi/src/octo_benchmark.cpp)
ament_target_dependencies(octo_benchmark rclcpp std_msgs octomap_msgs octomap) 



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  octo_publisher
  octo_planner
  octo_benchmark
  DESTINATION lib/${PROJECT_NAME})

ament_package()
