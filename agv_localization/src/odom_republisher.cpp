#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomRepublisher : public rclcpp::Node
{
public:
  OdomRepublisher() : Node("odom_republisher_node")
  {
    // Use simulation time
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/diff_cont/odom_with_covariance", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diff_cont/odom", 10, std::bind(&OdomRepublisher::odomCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Odometry republisher started: /diff_cont/odom -> /diff_cont/odom_with_covariance");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    nav_msgs::msg::Odometry new_odom = *odom;
    
    // Add covariance for EKF - robot_localization requires non-zero covariance
    // Pose covariance (6x6 = 36 elements: x, y, z, roll, pitch, yaw)
    new_odom.pose.covariance[0] = 0.01;   // x variance
    new_odom.pose.covariance[7] = 0.01;   // y variance
    new_odom.pose.covariance[14] = 0.01;  // z variance
    new_odom.pose.covariance[21] = 0.01;  // roll variance
    new_odom.pose.covariance[28] = 0.01;  // pitch variance
    new_odom.pose.covariance[35] = 0.01;  // yaw variance
    
    // Twist covariance (6x6 = 36 elements: vx, vy, vz, vroll, vpitch, vyaw)
    new_odom.twist.covariance[0] = 0.01;   // vx variance
    new_odom.twist.covariance[7] = 0.01;   // vy variance
    new_odom.twist.covariance[14] = 0.01;  // vz variance
    new_odom.twist.covariance[21] = 0.01;  // vroll variance
    new_odom.twist.covariance[28] = 0.01;  // vpitch variance
    new_odom.twist.covariance[35] = 0.01;  // vyaw variance
    
    odom_pub_->publish(new_odom);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRepublisher>());
  rclcpp::shutdown();
  return 0;
}
