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
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)

# Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Set AUTOUIC search paths for .ui files
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}  # Pour les fichiers générés par UIC
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
)

set(HEADER_FILES
  include/curobo_robot_setup/curobo_robot_setup_panel.hpp
  include/curobo_robot_setup/sphere_manager.hpp
  include/curobo_robot_setup/curobo_config.hpp
  include/curobo_robot_setup/joint_config_dialog.hpp
)

# Create library
add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES}
  ${HEADER_FILES}
  ${UI_FILES}
)

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  pluginlib
  urdf
  std_msgs
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  yaml-cpp
)

# Install library
install(TARGETS ${PROJECT_NAME}
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

# Export plugin
pluginlib_export_plugin_description_file(rviz_common plugin.xml)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
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