#pragma once
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_gz_interfaces/srv/control_world.hpp>
#include <thread>
#include <atomic>

class RosWorker : public QObject {
    Q_OBJECT
public:
    explicit RosWorker(QObject* parent = nullptr);
    ~RosWorker();

    void start(); // Start spinning
    void stop();  // Stop spinning

    // "Used controls to start or stop the sim"
    void setSimulationPaused(bool paused);

signals:
    // Thread-safe signal to update GUI
    void droneStateReceived(QString id, double x, double y, double z, double vx, double vy, double vz);

private:
    void spinLoop(); // The background thread loop

    std::atomic<bool> running_;
    std::thread spinThread_;
    rclcpp::Node::SharedPtr node_;
    
    // Subscriptions for multiple drones
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subs_;
    
    // Client for world control
    rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr controlClient_;
};
