// ============================================================================
// camera_zed2.cpp  - Publish ZED2 Left Camera (VGA 672×376 @ 30FPS)
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class CameraZED2 : public rclcpp::Node
{
public:
    CameraZED2()
    : Node("camera_zed2")
    {
        // Declare parameters
        device_ = declare_parameter<std::string>("device", "/dev/video4");
        frame_id_ = declare_parameter<std::string>("frame_id", "zed2_left_camera_frame");

        // Open camera
        cap_.open(device_, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_FATAL(get_logger(), "❌ Cannot open camera device %s", device_.c_str());
            rclcpp::shutdown();
            return;
        }

        // Configure ZED2 UVC
        cap_.set(cv::CAP_PROP_FRAME_WIDTH,  672);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 376);
        cap_.set(cv::CAP_PROP_FPS, 30);

        RCLCPP_INFO(get_logger(),
            "📸 ZED2 camera opened at %s (672×376 @ 30 FPS)", device_.c_str());

        pub_image_ = create_publisher<sensor_msgs::msg::Image>("/zed2/image_raw", 10);

        // Timer 30FPS
        timer_ = create_wall_timer(
            33ms,
            std::bind(&CameraZED2::grabFrame, this));
    }

private:
    void grabFrame()
    {
        cv::Mat img;
        cap_ >> img;
        if (img.empty()) return;

        // Convert to ROS Image msg
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            img
        ).toImageMsg();

        msg->header.stamp = now();
        msg->header.frame_id = frame_id_;

        pub_image_->publish(*msg);
    }

    // Members
    cv::VideoCapture cap_;
    std::string device_, frame_id_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraZED2>());
    rclcpp::shutdown();
    return 0;
}
