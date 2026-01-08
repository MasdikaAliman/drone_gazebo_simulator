cmake_minimum_required(VERSION 3.8)
project(drone_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(NODE_BEHAVIOR_SOURCE
   src/GoToPose.cpp
)


# find dependencies
# include(include/drone_behavior)
include_directories(include/drone_behavior)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(drone_behavior src/drone_behavior.cpp ${NODE_BEHAVIOR_SOURCE})


ament_target_dependencies(
  
  drone_behavior
  behaviortree_cpp
  rclcpp
  rclcpp_action
  drone_msgs
  nav_msgs
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  drone_behavior
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
