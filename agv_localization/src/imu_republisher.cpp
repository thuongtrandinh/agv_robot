#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IMURepublisher : public rclcpp::Node
{
public:
  IMURepublisher() : Node("imu_republisher_node")
  {
    // Use simulation time
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_with_covariance", 10);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&IMURepublisher::imuCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "IMU republisher started: /imu -> /imu_with_covariance");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
  {
    sensor_msgs::msg::Imu new_imu = *imu;
    
    // Add covariance for EKF - robot_localization requires non-zero covariance
    // Orientation covariance (not used by EKF, set to -1 to mark as unused)
    new_imu.orientation_covariance[0] = -1.0;
    new_imu.orientation_covariance[4] = -1.0;
    new_imu.orientation_covariance[8] = -1.0;
    
    // Angular velocity covariance (using yaw rate - last element)
    new_imu.angular_velocity_covariance[0] = 0.001;  // roll rate variance
    new_imu.angular_velocity_covariance[4] = 0.001;  // pitch rate variance
    new_imu.angular_velocity_covariance[8] = 0.001;  // yaw rate variance
    
    // Linear acceleration covariance (using x acceleration)
    new_imu.linear_acceleration_covariance[0] = 0.01;  // ax variance
    new_imu.linear_acceleration_covariance[4] = 0.01;  // ay variance
    new_imu.linear_acceleration_covariance[8] = 0.01;  // az variance
    
    imu_pub_->publish(new_imu);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMURepublisher>());
  rclcpp::shutdown();
  return 0;
}