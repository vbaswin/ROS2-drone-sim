#include "RosWorker.h"
#include <chrono>
#include <memory>
#include <ros_gz_interfaces/srv/detail/control_world__struct.hpp>
#include <thread>

RosWorker::RosWorker(QObject *parent) : QObject(parent), running_(false) {}

RosWorker::~RosWorker() { stop(); }

void RosWorker::start() {
    if (running_)
        return;
    running_ = true;

    // Create the ROS Node
    node_ = rclcpp::Node::make_shared("viz_client_node");

    // "Setup a few drones" - We subscribe to a fixed list for this learning project.
    // In a production app, we would use topic discovery.
    std::vector<std::string> droneNames = {"drone1", "drone2", "drone3"};

    // Quality of Service: Best Effort is preferred for live visualization telemetry
    auto qos = rclcpp::SensorDataQoS();

    for (const auto &name : droneNames) {
        std::string topic = "/model/" + name + "/odometry";
        auto sub = node_->create_subscription<nav_msgs::msg::Odometry>(
            topic, qos,
            [this, name](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // Extract data
                double x = msg->pose.pose.position.x;
                double y = msg->pose.pose.position.y;
                double z = msg->pose.pose.position.z;
                double vx = msg->twist.twist.linear.x;
                double vy = msg->twist.twist.linear.y;
                double vz = msg->twist.twist.linear.z;

                // Emit signal (Thread-safe via Qt QueuedConnection)
                emit droneStateReceived(QString::fromStdString(name), x, y, z, vx, vy, vz);
            });
        subs_.push_back(sub);
    }

    // Setup Service Client for Simulation Control
    controlClient_ = node_->create_client<ros_gz_interfaces::srv::ControlWorld>("/world/default/control");

    // Launch the thread
    spinThread_ = std::thread(&RosWorker::spinLoop, this);
}

void RosWorker::stop() {
    running_ = false;
    if (spinThread_.joinable())
        spinThread_.join();
}

void RosWorker::spinLoop() {
    while (running_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void RosWorker::setSimulationPaused(bool paused) {
    // if (!controlClient_->wait_for_service(std::chrono::seconds(1)))
    // if (!controlClient_->service_is_ready()) {
    //     RCLCPP_WARN(node_->get_logger(), "Control service not available");
    //     return;
    //
    //     auto req = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
    //     req->world_control.pause = paused;
    //
    //     // Asynchronous call to avoid blocking the GUI thread
    //     controlClient_->async_send_request(req);
    // }

    // moveing to background thread
    std::thread([this, paused]() {
        if (!controlClient_->wait_for_service(std::chrono::seconds(1)))
            return;
        auto req = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
        req->world_control.pause = paused;
        controlClient_->async_send_request(req);
    }).detach();
}
