#pragma once

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <atomic>

class KeyboardInput : public rclcpp::Node
{
public:
    KeyboardInput();
    ~KeyboardInput();
    void publishCmd(double linear, double angular);
    std::atomic<bool> running_;

private:
    void keyboardLoop();

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    std::thread input_thread_;
    struct termios oldt_;
};