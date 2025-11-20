#include "agv_controller/KeyboardInput.h"
#include <iostream>
#include <csignal>
#include <atomic>
#include <cmath>

// Global pointer for signal handler
static KeyboardInput* g_keyboard_input_ptr = nullptr;

void sigintHandler(int signum)
{
    if (g_keyboard_input_ptr) {
        g_keyboard_input_ptr->publishCmd(0.0, 0.0);
        g_keyboard_input_ptr->running_ = false;
    }
    std::cout << "\n[INFO] Ctrl+C pressed. Robot stopped. Exiting...\n";
    std::exit(0);
}

KeyboardInput::KeyboardInput() : Node("keyboard_input"), running_(true), publish_enabled_(true)
{
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/diff_cont/cmd_vel", 10);

    // Initialize speeds
    linear_speed_ = 0.0;
    angular_speed_ = 0.0;

    // Set terminal to raw mode
    tcgetattr(STDIN_FILENO, &oldt_);
    struct termios newt = oldt_;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Set global pointer for SIGINT
    g_keyboard_input_ptr = this;
    std::signal(SIGINT, sigintHandler);

    input_thread_ = std::thread(&KeyboardInput::keyboardLoop, this);

    RCLCPP_INFO(
        this->get_logger(),
        "Keyboard teleop started.\n"
        "W/S = tăng / giảm tốc tiến\n"
        "A/D = tăng / giảm tốc quay\n"
        "C = bật / tắt gửi lệnh vận tốc (toggle publish)\n"
        "Q = thoát\n"
        "Ctrl+C = dừng robot và thoát."
    );
}

KeyboardInput::~KeyboardInput()
{
    running_ = false;
    if (input_thread_.joinable())
        input_thread_.join();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    g_keyboard_input_ptr = nullptr;
}

void KeyboardInput::keyboardLoop()
{
    char c;

    const double linear_step = 0.1;   // step tăng tốc
    const double min_linear = 0.2;    // minimum linear speed when starting from 0
    const double angular_step = 0.1;  // step tăng góc
    const double max_linear = 1.0;    // giới hạn tốc độ tiến
    const double max_angular = 1.0;   // giới hạn tốc độ xoay

    while (running_)
    {
        c = getchar();

        if (c == 'q' || c == 'Q') {
            running_ = false;
            break;
        }

        switch (c)
        {
        case 'w':
        case 'W':
            if (std::fabs(linear_speed_) < 1e-6) {
                // start from minimum linear speed instead of the small step
                linear_speed_ = min_linear;
            } else {
                linear_speed_ += linear_step;
                if (linear_speed_ > max_linear)
                    linear_speed_ = max_linear;
            }
            break;

        case 's':
        case 'S':
            if (std::fabs(linear_speed_) < 1e-6) {
                // start moving backwards from -min_linear
                linear_speed_ = -min_linear;
            } else {
                linear_speed_ -= linear_step;
                if (linear_speed_ < -max_linear)
                    linear_speed_ = -max_linear;
            }
            break;

        case 'a':
        case 'A':
            angular_speed_ += angular_step;
            if (angular_speed_ > max_angular)
                angular_speed_ = max_angular;
            break;

        case 'd':
        case 'D':
            angular_speed_ -= angular_step;
            if (angular_speed_ < -max_angular)
                angular_speed_ = -max_angular;
            break;

        case 'c':
        case 'C':
            // Toggle publishing on/off. When turning off, send one zero-velocity
            // message to ensure robot stops, then stop publishing further commands.
            if (publish_enabled_) {
                publishCmd(0.0, 0.0);
                publish_enabled_ = false;
                std::cout << "\n[INFO] Publishing disabled (C pressed)." << std::endl;
            } else {
                publish_enabled_ = true;
                std::cout << "\n[INFO] Publishing enabled (C pressed)." << std::endl;
            }
            break;

        default:
            break;
        }

        if (publish_enabled_) {
            publishCmd(linear_speed_, angular_speed_);
        }

        std::cout << "\rLinear: " << linear_speed_
                  << " m/s | Angular: " << angular_speed_
                  << " rad/s     " << std::flush;
    }
}

void KeyboardInput::publishCmd(double linear, double angular)
{
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";

    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;

    cmd_pub_->publish(msg);
}
