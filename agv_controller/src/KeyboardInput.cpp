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

KeyboardInput::KeyboardInput() : Node("keyboard_input"), running_(true)
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
        "C = dừng robot\n"
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

    const double fixed_linear_speed = 0.3;   // Vận tốc tiến/lùi cố định
    const double fixed_angular_speed = 0.3;  // Vận tốc xoay cố định

    while (running_)
    {
        c = getchar();

        if (c == 'q' || c == 'Q') {
            publishCmd(0.0, 0.0); // Dừng robot trước khi thoát
            running_ = false;
            break;
        }

        switch (c)
        {
        case 'w':
        case 'W':
            linear_speed_ = fixed_linear_speed;
            break;

        case 's':
        case 'S':
            linear_speed_ = -fixed_linear_speed;
            break;

        case 'a':
        case 'A':
            angular_speed_ = fixed_angular_speed;
            break;

        case 'd':
        case 'D':
            angular_speed_ = -fixed_angular_speed;
            break;

        case 'c':
        case 'C':
            linear_speed_  = 0.0;
            angular_speed_ = 0.0;
            break;

        default:
            break;
        }

        publishCmd(linear_speed_, angular_speed_);

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
