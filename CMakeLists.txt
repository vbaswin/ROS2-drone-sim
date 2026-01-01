cmake_minimum_required(VERSION 3.16)
project(DronePathSim)

# Enforce C++17 for ROS 2 Humble and Qt 6 compatibility
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# Dependency Discovery
# -----------------------------------------------------------------------------

# ROS 2 Core Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED) # For Simulation Control

# Qt 6 Components (Source built paths found via env vars)
find_package(Qt6 COMPONENTS Widgets Core Gui OpenGLWidgets REQUIRED)

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
  REQUIRED
)

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

# Include Directories
target_include_directories(drone_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# -----------------------------------------------------------------------------
# Linking
# -----------------------------------------------------------------------------

target_link_libraries(drone_sim
  Qt6::Widgets
  Qt6::OpenGLWidgets
  VTK::CommonCore
  VTK::GUISupportQt
  VTK::RenderingOpenGL2
  VTK::IOImage
  VTK::IOMovie
  rclcpp::rclcpp
  nav_msgs::nav_msgs__rosidl_generator_c
  nav_msgs::nav_msgs__rosidl_typesupport_fastrtps_c
  geometry_msgs::geometry_msgs__rosidl_generator_c
  geometry_msgs::geometry_msgs__rosidl_typesupport_fastrtps_c
  ros_gz_interfaces::ros_gz_interfaces__rosidl_generator_c
  ros_gz_interfaces::ros_gz_interfaces__rosidl_typesupport_fastrtps_c
)

# Qt Meta-Object Compiler configuration
set_target_properties(drone_sim PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTORCC ON
)

# VTK 9 Module Initialization (Critical for static/hybrid builds)
vtk_module_autoinit(
  TARGETS drone_sim
  MODULES VTK::RenderingOpenGL2 VTK::GUISupportQt
)

# -----------------------------------------------------------------------------
# Install Rules
# -----------------------------------------------------------------------------
install(TARGETS drone_sim DESTINATION lib/${PROJECT_NAME})
ament_package()
