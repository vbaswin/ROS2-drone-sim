/**
 * @file DroneKeyboardTeleop.cpp
 * @brief Custom keyboard teleop with vim-like bindings and multi-drone selection.
 *
 * Build: g++ -o drone_teleop DroneKeyboardTeleop.cpp $(pkg-config --cflags --libs rclcpp
geometry_msgs) -std=c++17
 * Or add to CMakeLists.txt as a separate executable.
 *
 * Keybindings (vim-inspired):
 *   Movement:     h = left, l = right, j = backward, k = forward
 *   Altitude:     u = up, o = down
 *   Rotation:     a = yaw left, d = yaw right
 *   Drone Select: 1 = drone1, 2 = drone2, 3 = drone3
 *   Speed:        + = increase, - = decrease
 *   Quit:         q or Ctrl+C
 */

#include <csignal>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <termios.h>
#include <unistd.h>

// Terminal raw mode handling
class TerminalRawMode {
private:
    struct termios original_;

public:
    TerminalRawMode() {
        tcgetattr(STDIN_FILENO, &original_);
        struct termios raw = original_;
        raw.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
        raw.c_cc[VMIN] = 0;              // Non-blocking
        raw.c_cc[VTIME] = 0;             // No timeout
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
    ~TerminalRawMode() {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }

    TerminalRawMode(const TerminalRawMode &) = delete;
    TerminalRawMode &operator=(const TerminalRawMode &) = delete;
};

class DroneKeyboardTeleop : public rclcpp::Node {
private:
    std::map<int, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers_;
    int current_drone_;
    double linear_speed_;
    double angular_speed_;

    // For HOLD behavior
    geometry_msgs::msg::Twist current_twist_;
    std::chrono::steady_clock::time_point last_key_time_;
    static constexpr int KEY_TIMEOUT_MS = 150; // Stop after 150ms of no input

    void selectDrone(int drone_id) {
        current_drone_ = drone_id;
        std::cout << "\r\033[K";
        std::cout << "\033[1;32m>>> Selected: DRONE " << drone_id << " <<<\033[0m";
        std::cout << " (";
        switch (drone_id) {
        case 1:
            std::cout << "\033[1;31mRED\033[0m";
            break;
        case 2:
            std::cout << "\033[1;32mGREEN\033[0m";
            break;
        case 3:
            std::cout << "\033[1;34mBLUE\033[0m";
            break;
        }
        std::cout << ")\n"
                  << std::flush;
    }

    void printSpeed() {
        std::cout << "\rSpeed: linear=" << linear_speed_
                  << " angular=" << angular_speed_ << "   \n"
                  << std::flush;
    }

    void printHelp() {
        std::cout << R"(
  ╔══════════════════════════════════════════════════════════════╗
  ║        DRONE KEYBOARD TELEOP (Vim-style + HOLD)              ║
  ╠══════════════════════════════════════════════════════════════╣
  ║  MOVEMENT (HOLD)   ALTITUDE (HOLD)  ROTATION (HOLD)          ║
  ║  ──────────────    ──────────────   ─────────────            ║
  ║      k               u (up)          a ↺  ↻ d                ║
  ║    h   l                                                     ║
  ║      j               o (down)                                ║
  ║                                                              ║
  ╠══════════════════════════════════════════════════════════════╣
  ║  DRONE SELECT      SPEED            OTHER                    ║
  ║  ────────────      ─────            ─────                    ║
  ║  1 = Drone1 (RED)  + = faster       SPACE = stop             ║
  ║  2 = Drone2 (GRN)  - = slower       q = quit                 ║
  ║  3 = Drone3 (BLU)                                            ║
  ╚══════════════════════════════════════════════════════════════╝
          >>> HOLD keys for continuous movement <<<
  )";
        selectDrone(1);
    }

public:
    DroneKeyboardTeleop()
        : Node("drone_keyboard_teleop"), current_drone_(1), linear_speed_(1.5), angular_speed_(1.5), last_key_time_(std::chrono::steady_clock::now()) {
        for (int i = 1; i <= 3; ++i) {
            std::string topic = "/drone" + std::to_string(i) + "/cmd_vel";
            publishers_[i] = create_publisher<geometry_msgs::msg::Twist>(topic, 10);
        }
        printHelp();
    }

    void run() {
        TerminalRawMode raw_mode;

        rclcpp::Rate rate(50); // 50 Hz for responsive control

        while (rclcpp::ok()) {
            char c = 0;
            bool key_pressed = (read(STDIN_FILENO, &c, 1) > 0);

            if (key_pressed) {
                last_key_time_ = std::chrono::steady_clock::now();

                // Process the key
                bool movement_key = true;
                geometry_msgs::msg::Twist new_twist;

                switch (c) {
                // === MOVEMENT (vim-like) ===
                case 'k':
                    new_twist.linear.x = linear_speed_;
                    break;
                case 'j':
                    new_twist.linear.x = -linear_speed_;
                    break;
                case 'h':
                    new_twist.linear.y = linear_speed_;
                    break;
                case 'l':
                    new_twist.linear.y = -linear_speed_;
                    break;

                // === ALTITUDE ===
                case 'u':
                    new_twist.linear.z = linear_speed_;
                    break;
                case 'o':
                    new_twist.linear.z = -linear_speed_;
                    break;

                // === ROTATION ===
                case 'a':
                    new_twist.angular.z = angular_speed_;
                    break;
                case 'd':
                    new_twist.angular.z = -angular_speed_;
                    break;

                // === DRONE SELECTION ===
                case '1':
                    selectDrone(1);
                    movement_key = false;
                    break;
                case '2':
                    selectDrone(2);
                    movement_key = false;
                    break;
                case '3':
                    selectDrone(3);
                    movement_key = false;
                    break;

                // === SPEED CONTROL ===
                case '+':
                case '=':
                    linear_speed_ = std::min(linear_speed_ * 1.2, 5.0);
                    angular_speed_ = std::min(angular_speed_ * 1.2, 5.0);
                    printSpeed();
                    movement_key = false;
                    break;
                case '-':
                case '_':
                    linear_speed_ = std::max(linear_speed_ * 0.8, 0.1);
                    angular_speed_ = std::max(angular_speed_ * 0.8, 0.1);
                    printSpeed();
                    movement_key = false;
                    break;

                // === QUIT ===
                case 'q':
                case 3:
                    // Send stop before exiting
                    current_twist_ = geometry_msgs::msg::Twist();
                    publishers_[current_drone_]->publish(current_twist_);
                    std::cout << "\nExiting...\n";
                    return;

                // === IMMEDIATE STOP ===
                case ' ':
                    new_twist = geometry_msgs::msg::Twist();
                    current_twist_ = new_twist;
                    break;

                default:
                    movement_key = false;
                    break;
                }

                if (movement_key) {
                    current_twist_ = new_twist;
                }
            }

            // Check for key timeout (HOLD behavior)
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                                 last_key_time_)
                               .count();

            if (elapsed > KEY_TIMEOUT_MS) {
                // Key released (timeout) - stop
                current_twist_ = geometry_msgs::msg::Twist();
            }

            // Always publish current twist (enables smooth HOLD behavior)
            publishers_[current_drone_]->publish(current_twist_);

            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneKeyboardTeleop>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
