#ifndef KEYBOARD_INPUT_H
#define KEYBOARD_INPUT_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <termios.h>
#include <atomic>
#include <thread>

class KeyboardInput : public rclcpp::Node
{
public:
    KeyboardInput();
    ~KeyboardInput();

    void publishCmd(double linear, double angular);

    std::atomic<bool> running_;
    std::atomic<bool> publish_enabled_;

private:
    void keyboardLoop();

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

    std::thread input_thread_;

    struct termios oldt_;

    double linear_speed_;
    double angular_speed_;
};

#endif
