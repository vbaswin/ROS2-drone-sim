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
        raw.c_cc[VTIME] = 1;             // 100ms timeout
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
    ~TerminalRawMode() {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
};
class DroneKeyboardTeleop : public rclcpp::Node {
public:
    DroneKeyboardTeleop() : Node("drone_keyboard_teleop"), current_drone_(1) {
        // Create publishers for all drones
        for (int i = 1; i <= 3; ++i) {
            std::string topic = "/drone" + std::to_string(i) + "/cmd_vel";
            publishers_[i] = create_publisher<geometry_msgs::msg::Twist>(topic, 10);
        }

        printHelp();
    }

    void run() {
        TerminalRawMode raw_mode; // RAII: restores terminal on exit

        rclcpp::Rate rate(20); // 20 Hz control loop
        geometry_msgs::msg::Twist twist;

        while (rclcpp::ok()) {
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                // Reset twist
                twist = geometry_msgs::msg::Twist();

                bool valid_key = true;
                switch (c) {
                // === MOVEMENT (vim-like) ===
                case 'k':
                    twist.linear.x = linear_speed_;
                    break; // Forward
                case 'j':
                    twist.linear.x = -linear_speed_;
                    break; // Backward
                case 'h':
                    twist.linear.y = linear_speed_;
                    break; // Strafe Left
                case 'l':
                    twist.linear.y = -linear_speed_;
                    break; // Strafe Right

                // === ALTITUDE ===
                case 'u':
                    twist.linear.z = linear_speed_;
                    break; // Up
                case 'o':
                    twist.linear.z = -linear_speed_;
                    break; // Down

                // === ROTATION ===
                case 'a':
                    twist.angular.z = angular_speed_;
                    break; // Yaw Left
                case 'd':
                    twist.angular.z = -angular_speed_;
                    break; // Yaw Right

                // === DRONE SELECTION ===
                case '1':
                    selectDrone(1);
                    valid_key = false;
                    break;
                case '2':
                    selectDrone(2);
                    valid_key = false;
                    break;
                case '3':
                    selectDrone(3);
                    valid_key = false;
                    break;

                // === SPEED CONTROL ===
                case '+':
                case '=':
                    linear_speed_ *= 1.1;
                    angular_speed_ *= 1.1;
                    printSpeed();
                    valid_key = false;
                    break;
                case '-':
                case '_':
                    linear_speed_ *= 0.9;
                    angular_speed_ *= 0.9;
                    printSpeed();
                    valid_key = false;
                    break;

                // === QUIT ===
                case 'q':
                case 3: // Ctrl+C
                    std::cout << "\nExiting...\n";
                    return;

                // === STOP (spacebar) ===
                case ' ':
                    twist = geometry_msgs::msg::Twist(); // All zeros = stop
                    break;

                default:
                    valid_key = false;
                    break;
                }

                if (valid_key) {
                    publishers_[current_drone_]->publish(twist);
                }
            } else {
                // No key pressed - send stop command to prevent drift
                twist = geometry_msgs::msg::Twist();
                publishers_[current_drone_]->publish(twist);
            }

            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

private:
    void selectDrone(int drone_id) {
        current_drone_ = drone_id;
        std::cout << "\r\033[K"; // Clear line
        std::cout << "\033[1;32m>>> Selected: DRONE " << drone_id << " <<<\033[0m" << std::flush;
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
        std::cout << ")\n";
    }

    void printSpeed() {
        std::cout << "\rSpeed: linear=" << linear_speed_
                  << " angular=" << angular_speed_ << "   \n";
    }

    void printHelp() {
        std::cout << R"(                                                                     
  ╔══════════════════════════════════════════════════════════════╗
  ║           DRONE KEYBOARD TELEOP (Vim-style)                  ║
  ╠══════════════════════════════════════════════════════════════╣
  ║  MOVEMENT          ALTITUDE         ROTATION                 ║
  ║  ─────────         ────────         ────────                 ║
  ║      k               u (up)          a ↺  ↻ d                ║
  ║    h   l                                                     ║
  ║      j               o (down)                                ║
  ║  (←↓↑→)                                                      ║
  ╠══════════════════════════════════════════════════════════════╣
  ║  DRONE SELECT      SPEED            OTHER                    ║
  ║  ────────────      ─────            ─────                    ║
  ║  1 = Drone1 (RED)  + = faster       SPACE = stop             ║
  ║  2 = Drone2 (GRN)  - = slower       q = quit                 ║
  ║  3 = Drone3 (BLU)                                            ║
  ╚══════════════════════════════════════════════════════════════╝
  )";
        selectDrone(1); // Default to drone 1
    }

    std::map<int, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers_;
    int current_drone_;
    double linear_speed_ = 1.0;
    double angular_speed_ = 1.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneKeyboardTeleop>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
