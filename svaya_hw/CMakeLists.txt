cmake_minimum_required(VERSION 3.8)
project(svaya_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

#find_package(knexo_ethercat REQUIRED)

set(knexo_ethercat_DIR "${CMAKE_SOURCE_DIR}/../../install/knexo_ethercat/share/knexo_ethercat/cmake/")
       message("knexo_ethercat dir ", ${CMAKE_PREFIX_PATH}, ${knexo_ethercat_DIR} )
       find_library(knexo_ethercat
           NAMES knexo_ethercat
           HINTS "${CMAKE_SOURCE_DIR}/../../install/knexo_ethercat/lib"
       )
#set(SVAYA_DRIVER_DIR "$ENV{HOME}/svaya_driver")
include_directories("${CMAKE_SOURCE_DIR}/../../install/knexo_ethercat/include")


set(ETHERCAT_DRIVER_DIR "${CMAKE_SOURCE_DIR}/../knexo_ethercat")

set(SVAYA_DRIVER_DIR "$ENV{HOME}/svaya_driver_v5") # set location of the .so file

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(SVAYA_DRIVER_LIB SvayaV5 HINTS ${SVAYA_DRIVER_DIR}/lib)

link_directories(
/opt/etherlab/lib/
)

include_directories(include
           /opt/etherlab/include)

include_directories(${SVAYA_DRIVER_DIR}/include)

include_directories(${ETHERCAT_DRIVER_DIR}/include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/svaya_hw.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${SVAYA_DRIVER_LIB}
    ${knexo_ethercat}
    )

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${ETHERCAT_DRIVER_DIR}/include

)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_manager

)

# Export hardware pligins
pluginlib_export_plugin_description_file(hardware_interface svaya_hw_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
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

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
#  knexo_ethercat
)

ament_package()
