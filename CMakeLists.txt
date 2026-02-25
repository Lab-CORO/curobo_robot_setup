cmake_minimum_required(VERSION 3.8)
project(curobo_robot_setup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Set AUTOUIC search paths for .ui files
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# ──────────────────────────────────────────────────────────────────────────────
# Generate ROS2 service interfaces
# ──────────────────────────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GenerateSpheres.srv"
  DEPENDENCIES std_msgs
)

# ──────────────────────────────────────────────────────────────────────────────
# Include directories
# ──────────────────────────────────────────────────────────────────────────────
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# UI files
set(UI_FILES
  ui/curobo_robot_setup.ui
)

# Source files
set(SOURCE_FILES
  src/curobo_robot_setup_panel.cpp
  src/sphere_manager.cpp
  src/curobo_config.cpp
  src/joint_config_dialog.cpp
  src/collision_config_dialog.cpp
)

set(HEADER_FILES
  include/curobo_robot_setup/curobo_robot_setup_panel.hpp
  include/curobo_robot_setup/sphere_manager.hpp
  include/curobo_robot_setup/curobo_config.hpp
  include/curobo_robot_setup/joint_config_dialog.hpp
  include/curobo_robot_setup/collision_config_dialog.hpp
)

# Create library (named _plugin to avoid conflict with rosidl target of same name)
set(PLUGIN_LIB ${PROJECT_NAME}_plugin)

add_library(${PLUGIN_LIB} SHARED
  ${SOURCE_FILES}
  ${HEADER_FILES}
  ${UI_FILES}
)

# Dependencies
ament_target_dependencies(${PLUGIN_LIB}
  rclcpp
  interactive_markers
  geometry_msgs
  visualization_msgs
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  pluginlib
  urdf
  std_msgs
)

# Link against generated service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PLUGIN_LIB}
  Qt5::Widgets
  yaml-cpp
  tinyxml2::tinyxml2
  "${cpp_typesupport_target}"
)

# ──────────────────────────────────────────────────────────────────────────────
# Standalone node: urdf_extractor
# ──────────────────────────────────────────────────────────────────────────────
add_executable(urdf_extractor src/urdf_extractor_node.cpp)

ament_target_dependencies(urdf_extractor rclcpp std_msgs)

target_link_libraries(urdf_extractor tinyxml2::tinyxml2)

install(TARGETS urdf_extractor
  DESTINATION lib/${PROJECT_NAME}
)

# ──────────────────────────────────────────────────────────────────────────────
# Install library
# ──────────────────────────────────────────────────────────────────────────────
install(TARGETS ${PLUGIN_LIB}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install UI files
install(DIRECTORY ui/
  DESTINATION share/${PROJECT_NAME}/ui
)

# Install plugin description
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install rviz config files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install Python scripts
install(PROGRAMS
  src/sphere_fit_service.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(rviz_common plugin.xml)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PLUGIN_LIB})
ament_export_dependencies(
  rclcpp
  interactive_markers
  geometry_msgs
  visualization_msgs
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  pluginlib
  urdf
  std_msgs
  Qt5
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
