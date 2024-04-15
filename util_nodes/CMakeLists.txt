cmake_minimum_required(VERSION 3.8)
project(util_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)



include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(fk src/fk.cpp)
ament_target_dependencies(fk rclcpp std_msgs sensor_msgs kdl_parser geometry_msgs nav_msgs urdf)

add_executable(cart_pos src/cart_pos.cpp)
ament_target_dependencies(cart_pos rclcpp std_msgs)

add_executable(torq src/torque.cpp)
ament_target_dependencies(torq rclcpp std_msgs urdf sensor_msgs)

add_executable(test1 src/test.cpp)
ament_target_dependencies(test1 rclcpp std_msgs urdf sensor_msgs)

install(TARGETS
  fk
  cart_pos
  torq
  test1
  DESTINATION lib/${PROJECT_NAME})



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

ament_package()
