cmake_minimum_required(VERSION 3.8)
project(svaya_trajectory_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(svaya_joint_state_reader REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(svaya_hw REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
rclcpp
trajectory_msgs
svaya_joint_state_reader
joint_trajectory_controller
controller_manager_msgs
#  rapidjson
)



add_library(
        ${PROJECT_NAME}
        SHARED
        src/trajectory_publisher.cpp)
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(
        ${PROJECT_NAME}
        rclcpp
        trajectory_msgs
        svaya_joint_state_reader
        joint_trajectory_controller
        controller_manager_msgs
 )


install(DIRECTORY include/
        DESTINATION include
)

#install(
#  DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
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


ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
rclcpp
trajectory_msgs
svaya_joint_state_reader
joint_trajectory_controller
controller_manager_msgs
)


ament_package()
