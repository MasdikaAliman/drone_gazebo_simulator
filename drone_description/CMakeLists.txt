cmake_minimum_required(VERSION 3.8)
project(drone_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(gz-cmake2 REQUIRED)
# find_package(gz-sim6 REQUIRED)
# find_package(gz-transport12 REQUIRED)
# find_package(gz-msgs9 REQUIRED)
# find_package(gz-plugin2 REQUIRED)
# find_package(gz-math7 REQUIRED)
# find_package(gz-common5 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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


# add_library(SetPosePlugin SHARED plugins/SetPosePlugin.cc)
# target_link_libraries(SetPosePlugin
#   gz-sim6::gz-sim6
#   gz-transport12::gz-transport12
#   gz-msgs9::gz-msgs9
#   gz-plugin2::gz-plugin2
#   gz-math7::gz-math7
#   gz-common5::gz-common5
# )

# add_library(RobotPosePublisher SHARED plugins/RobotPosePublisher.cc)
# target_link_libraries(RobotPosePublisher
#   gz-sim6::gz-sim6
#   gz-transport12::gz-transport12
#   gz-plugin2::gz-plugin2
#   gz-math7::gz-math7
#   gz-common5::gz-common5
# )

# install(
#   TARGETS SetPosePlugin RobotPosePublisher
#   LIBRARY DESTINATION lib
# )


install(
  DIRECTORY config launch meshes rviz urdf models world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()