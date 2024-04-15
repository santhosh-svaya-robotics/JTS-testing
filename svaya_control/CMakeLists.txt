cmake_minimum_required(VERSION 3.8)
project(svaya_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_trajectory_controller REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(SVAYA_DRIVER_DIR "$ENV{HOME}/svaya_driver_v5") # set location of the .so file

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(SVAYA_DRIVER_LIB SvayaV5 HINTS ${SVAYA_DRIVER_DIR}/lib/)

link_directories(
/opt/etherlab/lib/
)

include_directories(include
           /opt/etherlab/include)

include_directories(${SVAYA_DRIVER_DIR}/include)




add_executable(svaya_control_node src/svaya_launch.cpp)
target_link_libraries(svaya_control_node ${SVAYA_DRIVER_LIB})
ament_target_dependencies(svaya_control_node rclcpp controller_manager)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    svaya_control_node
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
