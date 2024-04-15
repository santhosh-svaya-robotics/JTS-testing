cmake_minimum_required(VERSION 3.8)
project(torq_predictor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
#include <std_msgs/msg/float64_multi_array.hpp>
set(EIGEN_DIR "/usr/include/eigen3/")
include_directories(include
    ${EIGEN_DIR})
add_executable(torq_predictor src/torq_predictor.cpp)
ament_target_dependencies(torq_predictor rclcpp std_msgs)

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN_DIR}
)

install(TARGETS
    torq_predictor
    DESTINATION lib/${PROJECT_NAME}
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

ament_package()
