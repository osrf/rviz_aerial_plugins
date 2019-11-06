cmake_minimum_required(VERSION 3.5)

project(rviz_aerial_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Tests currently only run on OS X @ OSRF jenkins
# Enable on Linux by providing a display, enable on Windows via EnableDisplayTests=True
option(EnableDisplayTests "EnableDisplayTests")
set(DisplayTests "False" CACHE STRING "DisplayTestsVariable")

if(DEFINED ENV{DISPLAY})
  set(DISPLAYPRESENT TRUE)
endif()

if(APPLE OR DISPLAYPRESENT OR EnableDisplayTests STREQUAL "True")
  message(STATUS "Enabling tests requiring a display")
else()
  set(SKIP_DISPLAY_TESTS "SKIP_TEST")
endif()

# options and directories for visual tests (see visual_testing_framework documentation)
option(EnableVisualTests "decides whether or not to enable the tests")

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_images)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_images)

# Run visual tests only if "EnableVisualTests=True"
if(EnableVisualTests STREQUAL "True")
  message(STATUS "Enabling visual tests")
else()
  set(SKIP_VISUAL_TESTS "SKIP_TEST")
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(proposed_aerial_msgs REQUIRED)

# These need to be added in the add_library() call so AUTOMOC detects them.
set(rviz_aerial_plugins_headers_to_moc
  include/rviz_aerial_plugins/displays/battery/battery_display.hpp
  include/rviz_aerial_plugins/displays/flight_info/flight_info_panel.hpp
  include/rviz_aerial_plugins/displays/flight_info/compass_widget.hpp
  include/rviz_aerial_plugins/displays/flight_info/attitude_display_indicator_widget.hpp
  include/rviz_aerial_plugins/displays/flight_info/vehicle_information_widget.hpp
  include/rviz_aerial_plugins/displays/goal3D/goal3D_display.hpp
)

set(rviz_aerial_plugins_source_files
  src/rviz_aerial_plugins/displays/battery/battery_display.cpp
  src/rviz_aerial_plugins/displays/flight_info/flight_info_panel.cpp
  src/rviz_aerial_plugins/displays/flight_info/compass_widget.cpp
  src/rviz_aerial_plugins/displays/flight_info/attitude_display_indicator_widget.cpp
  src/rviz_aerial_plugins/displays/flight_info/vehicle_information_widget.cpp
  src/rviz_aerial_plugins/displays/goal3D/goal3D_display.cpp
  src/rviz_aerial_plugins/utils/utils.cpp
)

add_library(rviz_aerial_plugins SHARED
  ${rviz_aerial_plugins_headers_to_moc}
  ${rviz_aerial_plugins_source_files}
)

target_include_directories(rviz_aerial_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rviz_rendering_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(rviz_aerial_plugins
  rviz_common::rviz_common
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_aerial_plugins PRIVATE "RVIZ_AERIAL_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(rviz_aerial_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(rviz_aerial_plugins
  rclcpp
  tf2
  interactive_markers
  std_msgs
  geometry_msgs
  visualization_msgs
  proposed_aerial_msgs
)

ament_export_include_directories(include)
ament_export_interfaces(rviz_aerial_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  rclcpp
)

install(
  TARGETS rviz_aerial_plugins
  EXPORT rviz_aerial_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
 DIRECTORY "${CMAKE_SOURCE_DIR}/icons"
 DESTINATION "share/${PROJECT_NAME}"
)

ament_package()
