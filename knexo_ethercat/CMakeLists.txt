cmake_minimum_required(VERSION 3.8)
project(knexo_ethercat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/Ethercat.msg"
# )


set(ETHERLAB_DIR /opt/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)

ament_export_include_directories(
  include
  ${ETHERLAB_DIR}/include
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  rosidl_default_generators
)




add_library(
        ${PROJECT_NAME}
        SHARED
        src/knexo_ethercat_master.cpp)
        
#add_library(knexo_ethercat src/knexo_ethercat_master.cpp)

#target_link_libraries(knexo_ethercat
#    ethercat
##    ${catkin_LIBRARIES}
#    )

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${ETHERLAB_DIR}/include
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${ETHERCAT_LIB})#target_link_libraries(knexo_ethercat ethercat)
ament_target_dependencies(
        ${PROJECT_NAME}
        rclcpp
	std_msgs
        rosidl_default_generators
)


#add_executable(controller_stopper_node src/controller_stopper_node.cpp src/controller_stopper.cpp)

#ament_target_dependencies(controller_stopper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

#add_executable(knexo_safety_node src/knexo_safety.cpp)

#ament_target_dependencies(knexo_safety_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

#add_executable(joint_space_planner src/joint_space_planner.cpp)

#ament_target_dependencies(joint_space_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})



install(DIRECTORY include/
        DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
  TARGETS ${PROJECT_NAME}
#  controller_stopper_node
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

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
        ${ETHERCAT_LIBRARY}
)
ament_export_dependencies(
  rclcpp
  std_msgs
  rosidl_default_generators
)


ament_package()

