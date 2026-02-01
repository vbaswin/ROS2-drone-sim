cmake_minimum_required(VERSION 3.16)
project(DronePathSim)

# Enforce C++17 for ROS 2 Humble and Qt 6 compatibility
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# Dependency Discovery
# -----------------------------------------------------------------------------

# ROS 2 Core Dependencies
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED) # For Simulation Control

# Qt 6 Components (Source built paths found via env vars)
find_package(Qt5 COMPONENTS Widgets Core Gui OpenGL REQUIRED)

# VTK 9.5 Components
find_package(VTK COMPONENTS
  CommonCore
  CommonDataModel
  FiltersSources
  InteractionStyle
  RenderingContextOpenGL2
  RenderingCore
  RenderingFreeType
  RenderingGL2PSOpenGL2
  RenderingOpenGL2
  GUISupportQt
  IOImage     # For PNGWriter
  IOMovie     # For AVIWriter
  IOOggTheora
  REQUIRED
)

# 2. Enable Unity Builds (Massive speedup for clean builds)
# Compiles multiple .cpp files as one single file to reduce overhead.
set(CMAKE_UNITY_BUILD ON)
set(CMAKE_UNITY_BUILD_BATCH_SIZE 32) # Group 32 files at a time

# -----------------------------------------------------------------------------
# Build Target
# -----------------------------------------------------------------------------

# Explicitly list ALL files here so they appear in project views
add_executable(drone_sim
  src/main.cpp
  src/MainWindow.h
  src/MainWindow.cpp
  src/RosWorker.h
  src/RosWorker.cpp
  src/DroneActor.h

)

# custom teleop
add_executable(drone_teleop
  src/DroneKeyboardTeleop.cpp
)

# Include Directories
target_include_directories(drone_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# -----------------------------------------------------------------------------
# Linking
# -----------------------------------------------------------------------------

target_link_libraries(drone_sim
Qt5::Widgets
Qt5::OpenGL
${VTK_LIBRARIES}
rclcpp::rclcpp
${nav_msgs_TARGETS}
${geometry_msgs_TARGETS}
${ros_gz_interfaces_TARGETS}
)

target_link_libraries(drone_teleop
rclcpp::rclcpp
${geometry_msgs_TARGETS}
)

# Qt Meta-Object Compiler configuration
set_target_properties(drone_sim PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTORCC ON
)

# 3. Precompiled Headers (PCH)
# Compile heavyweight headers (like Qt) ONCE and reuse them everywhere.
target_precompile_headers(drone_sim PRIVATE
    <QApplication>
    <QWidget>
    <QString>
    <vector>
    <memory>
)

# VTK 9 Module Initialization (Critical for static/hybrid builds)
vtk_module_autoinit(
  TARGETS drone_sim
  MODULES VTK::RenderingOpenGL2 VTK::GUISupportQt
)

# -----------------------------------------------------------------------------
#2j Install Rules
# -----------------------------------------------------------------------------
install(TARGETS drone_sim drone_teleop DESTINATION lib/${PROJECT_NAME})
