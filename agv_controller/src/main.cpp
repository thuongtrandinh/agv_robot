#include "agv_controller/KeyboardInput.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardInput>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
