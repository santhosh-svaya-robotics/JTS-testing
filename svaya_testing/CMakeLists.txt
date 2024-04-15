cmake_minimum_required(VERSION 3.8)
project(svaya_testing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-fno-sized-deallocation")
#set(CXXFLAGS="-fno-sized-deallocation")

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
set(CMAKE_CXX_STANDARD_REQUIRED true)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(svaya_trajectory_publisher REQUIRED)
find_package(svaya_joint_state_reader REQUIRED)
#find_package(svaya_hw REQUIRED)


find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#include_directories(include)
include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

include_directories(include)
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_definitions(${Qt5Widgets_DEFINITIONS})



set(THIS_PACKAGE_INCLUDE_DEPENDS
rclcpp
svaya_trajectory_publisher
trajectory_msgs
svaya_joint_state_reader

#svaya_hw
)

file(GLOB HDRS_UI RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "include/svaya_testing/ui/*.h")
file(GLOB SRC_UI RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/ui/*cpp")
file(GLOB UI_HDRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/ui/*.ui")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_wrap_ui(UIS_HDRS ${UI_HDRS})# ${QML_UI})

  # Generate moc files from Qt source files
qt5_wrap_cpp(MOC_SOURCES ${SRC_UI} ${HDRS_UI})

#include(${UIS_HDRS})
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

list(REMOVE_ITEM SRC_UI "src/ui/main.cpp")

add_library(
        ${PROJECT_NAME}
        SHARED
        ${SRC_UI}
        ${HDRS_UI}
        ${UIS_HDRS}
        ${MOC_SOURCES}
)
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${UIS_HDRS}
)
ament_target_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core Qt5::Gui)

add_executable(main src/main.cpp)

ament_target_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} svaya_joint_state_reader ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(torque_sensor src/torq_sensor.cpp)

ament_target_dependencies(torque_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(app_main src/ui/main.cpp ${SRC_UI} ${HDRS_UI} ${UIS_HDRS} ${MOC_SOURCES} ${CMAKE_CURRENT_BINARY_DIR}/ui_svayaui.h)
target_link_libraries(app_main Qt5::Widgets Qt5::Core Qt5::Gui)
set_property(TARGET app_main PROPERTY AUTOMOC ON)
ament_target_dependencies(app_main ${${PROJECT_NAME}_EXPORTED_TARGETS} svaya_joint_state_reader ${THIS_PACKAGE_INCLUDE_DEPENDS})

#install(DIRECTORY include/
 #       DESTINATION include
#)

install(
  TARGETS
  main
  torque_sensor
#  main2
app_main
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
)
ament_export_dependencies(
rclcpp
trajectory_msgs
svaya_joint_state_reader
svaya_trajectory_publisher
#svaya_hw
)

ament_package()
