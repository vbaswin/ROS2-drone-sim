 ---                                                                                       
  Suggested CLAUDE.md                                                                       
                                                                                            
  # CLAUDE.md                                                                               
                                                                                            
  This file provides guidance to Claude Code (claude.ai/code) when working with code in this
   repository.                                                                              
                                                                                            
  ## Project Overview                                                                       
                                                                                            
  **DronePathSim** is a C++17 ROS2 Humble application that visualizes multi-drone telemetry 
  in real-time using Qt 6 and VTK 9. It subscribes to Gazebo simulation odometry topics and 
  renders 3D flight paths with predictive trajectory vectors.                               
                                                                                            
  ## Build Commands                                                                         
                                                                                            
  ```bash                                                                                   
  # Prerequisites: ROS2 Humble environment must be sourced                                  
  source /opt/ros/humble/setup.bash                                                         
  # Or your custom ROS2 workspace overlay:                                                  
  # source ~/ros2_ws/install/setup.bash                                                     
                                                                                            
  # Configure (from project root)                                                           
  mkdir -p build && cd build                                                                
  cmake .. -DCMAKE_BUILD_TYPE=Release                                                       
                                                                                            
  # Build                                                                                   
  cmake --build . --parallel $(nproc)                                                       
                                                                                            
  # Run (requires active Gazebo simulation publishing to /model/<drone>/odometry)           
  ./drone_sim                                                                               
                                                                                            
  Debug Build                                                                               
                                                                                            
  mkdir -p build_debug && cd build_debug                                                    
  cmake .. -DCMAKE_BUILD_TYPE=Debug                                                         
  cmake --build . --parallel $(nproc)                                                       
                                                                                            
  Architecture                                                                              
                                                                                            
  Component Diagram                                                                         
                                                                                            
  ┌─────────────────────────────────────────────────────────────────┐                       
  │                         main.cpp                                │                       
  │  - Initializes ROS2 (rclcpp::init)                              │                       
  │  - Initializes Qt (QApplication)                                │                       
  │  - Sets VTK OpenGL surface format BEFORE QApplication           │                       
  └──────────────────────────┬──────────────────────────────────────┘                       
                             │                                                              
  ┌──────────────────────────▼──────────────────────────────────────┐                       
  │                      MainWindow                                 │                       
  │  - Owns QVTKOpenGLNativeWidget (VTK-Qt bridge)                  │                       
  │  - Owns vtkRenderer (3D scene graph)                            │                       
  │  - Owns RosWorker (ROS2 communications)                         │                       
  │  - Manages map<QString, DroneActor> for multi-drone tracking    │                       
  │  - Handles video recording via vtkAVIWriter                     │                       
  └────────────┬──────────────────────────────────┬─────────────────┘                       
               │                                  │                                         
  ┌────────────▼────────────┐      ┌──────────────▼──────────────────┐                      
  │       RosWorker         │      │          DroneActor             │                      
  │  - Runs rclcpp::spin    │      │  - Header-only VTK actor bundle │                      
  │    in dedicated thread  │      │  - Body: vtkSphereSource        │                      
  │  - Subscribes to        │      │  - Trail: vtkPolyData polyline  │                      
  │    /model/<name>/odom   │      │  - Prediction: vtkLineSource    │                      
  │  - Emits Qt signals     │      │    (kinematic extrapolation)    │                      
  │  - Controls Gazebo sim  │      └─────────────────────────────────┘                      
  │    via ControlWorld srv │                                                               
  └─────────────────────────┘                                                               
                                                                                            
  Threading Model                                                                           
                                                                                            
  - Main Thread (Qt Event Loop): UI, VTK rendering, signal/slot processing                  
  - ROS Spin Thread: RosWorker::spinLoop() runs rclcpp::spin() in a background std::thread  
  - Communication: Qt signals (droneStateReceived) cross thread boundaries via queued       
  connections                                                                               
                                                                                            
  Key Integration Points                                                                    
                                                                                            
  - Qt ↔ VTK: QVTKOpenGLNativeWidget requires QSurfaceFormat::setDefaultFormat() BEFORE     
  QApplication construction                                                                 
  - Qt ↔ ROS2: RosWorker inherits QObject to use signals; emits from ROS callbacks          
  (thread-safe via Qt's queued connections)                                                 
  - Gazebo Control: Uses ros_gz_interfaces::srv::ControlWorld service at                    
  /world/default/control                                                                    
                                                                                            
  ROS2 Topics & Services                                                                    
  ┌─────────┬────────────────────────┬────────────────────────────────────┐                 
  │  Type   │          Name          │            Message Type            │                 
  ├─────────┼────────────────────────┼────────────────────────────────────┤                 
  │ Sub     │ /model/drone1/odometry │ nav_msgs/msg/Odometry              │                 
  ├─────────┼────────────────────────┼────────────────────────────────────┤                 
  │ Sub     │ /model/drone2/odometry │ nav_msgs/msg/Odometry              │                 
  ├─────────┼────────────────────────┼────────────────────────────────────┤                 
  │ Sub     │ /model/drone3/odometry │ nav_msgs/msg/Odometry              │                 
  ├─────────┼────────────────────────┼────────────────────────────────────┤                 
  │ Service │ /world/default/control │ ros_gz_interfaces/srv/ControlWorld │                 
  └─────────┴────────────────────────┴────────────────────────────────────┘                 
  Dependencies                                                                              
                                                                                            
  - ROS2 Humble (rclcpp, nav_msgs, geometry_msgs, ros_gz_interfaces)                        
  - Qt 6 (Widgets, Core, Gui, OpenGLWidgets)                                                
  - VTK 9.5 (GUISupportQt, RenderingOpenGL2, IOImage, IOMovie)                              
                                                                                            
  Code Quality Standards                                                                    
                                                                                            
  When modifying this codebase, adhere to:                                                  
                                                                                            
  - C++17 standard (enforced by CMake)                                                      
  - SOLID Principles: Especially Single Responsibility (note how DroneActor only handles    
  visualization, RosWorker only handles ROS2)                                               
  - Const correctness: Prefer const for immutable data                                      
  - Smart pointers: Use vtkSmartPointer<T> for all VTK objects                              
  - Thread safety: All GUI updates must occur on the main thread; use Qt signals for        
  cross-thread communication
