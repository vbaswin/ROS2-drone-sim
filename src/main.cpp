#include <QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>
#include "MainWindow.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    // VTK 9 / Qt 6 Requirement:
    // Set the surface format before the QApplication is instantiated.
    // This ensures the OpenGL context supports the features VTK needs.
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Initialize Qt
    QApplication app(argc, argv);

    MainWindow window;
    window.resize(1024, 768);
    window.setWindowTitle("Drone Path Simulator (ROS2/Gazebo/VTK)");
    window.show();

    int result = app.exec();

    rclcpp::shutdown();
    return result;
}
