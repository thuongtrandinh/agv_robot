#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class MotorOdomNode : public rclcpp::Node {
public:
  MotorOdomNode() : Node("stm32_odom") {
    // Parameters (tune as needed)
    this->declare_parameter<double>("wheel_radius", 0.05);
    this->declare_parameter<double>("wheel_separation", 0.46);
    // Throttle STM32 high-frequency data (>100Hz) to reasonable rate
    this->declare_parameter<double>("odom_publish_rate", 50.0);
    // Topics and frames
    this->declare_parameter<std::string>("sensor_data_topic", "/sensor_data");
    this->declare_parameter<std::string>("odom_topic", "/diff_cont/odom");
    this->declare_parameter<std::string>("imu_topic", "/imu");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_footprint");

    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();

    double pub_rate = this->get_parameter("odom_publish_rate").as_double();
    if (pub_rate <= 0.0) pub_rate = 50.0;  // Default 50Hz for smooth real-time tracking
    odom_publish_period_ = rclcpp::Duration::from_seconds(1.0 / pub_rate);
    // Initialize to allow immediate first publish
    last_pub_time_ = this->now() - odom_publish_period_ - rclcpp::Duration::from_seconds(1.0);

    std::string sensor_topic = this->get_parameter("sensor_data_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    // Subscribe to unified sensor_data topic from STM32
    // Format: [gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, speed_left, speed_right]
    sensor_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      sensor_topic, 10, std::bind(&MotorOdomNode::sensorDataCallback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    last_time_ = this->now();

    // Reset odometry state on node startup
    resetOdometry();

    RCLCPP_INFO(this->get_logger(),
                "motor_odom started @ %.1f Hz. sensor_data_topic: %s (format: [gyro_x,y,z, acc_x,y,z, speed_L, speed_R])",
                pub_rate, sensor_topic.c_str());
  }

private:
  void resetOdometry() {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    last_gyro_z_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "Odometry state reset to initial values.");
  }

  void sensorDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    rclcpp::Time now = this->now();

    // Validate message format: [gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, speed_left, speed_right]
    if (msg->data.size() < 8) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "sensor_data size %zu < 8 (expected [gyro_xyz, acc_xyz, speed_L, speed_R])", 
                          msg->data.size());
      return;
    }

    // Extract IMU data (rad/s for gyro, m/s^2 for accel)
    double gyro_x = msg->data[0];
    double gyro_y = msg->data[1];
    double gyro_z = msg->data[2];
    double acc_x = msg->data[3];
    double acc_y = msg->data[4];
    double acc_z = msg->data[5];

    // Extract motor speeds (m/s linear velocity)
    double v_left = msg->data[6];
    double v_right = msg->data[7];

    // Compute dt for integration
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {  // Safety: reject huge dt spikes
      last_time_ = now;
      return;
    }

    // ========== ODOMETRY CALCULATION ==========
    // Compute robot velocities from wheel speeds
    double vx = (v_right + v_left) / 2.0;
    // FIXED: Negate to match encoder direction (left turn = positive angular velocity)
    double vtheta_encoder = -((v_right - v_left) / wheel_separation_);

    // Use IMU gyro_z for more accurate angular velocity (rad/s already)
    // Low-pass filter to smooth gyro noise: alpha=0.7 (trust gyro 70%, encoder 30%)
    const double alpha = 0.7;
    double vtheta_fused = alpha * gyro_z + (1.0 - alpha) * vtheta_encoder;

    // Integrate pose using fused angular velocity
    double dx = vx * dt;
    double dtheta = vtheta_fused * dt;

    yaw_ += dtheta;
    x_ += dx * std::cos(yaw_);
    y_ += dx * std::sin(yaw_);

    // Normalize yaw to [-pi, pi]
    while (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
    while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

    // ========== PUBLISH ODOMETRY ==========
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    // Use fused yaw from encoder + gyro_z
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vtheta_fused;

    // Covariance (conservative for STM32 hardware)
    for (int i = 0; i < 36; ++i) { 
      odom.pose.covariance[i] = 0.0; 
      odom.twist.covariance[i] = 0.0; 
    }
    odom.pose.covariance[0] = 1e-3;   // x
    odom.pose.covariance[7] = 1e-3;   // y
    odom.pose.covariance[35] = 5e-3;  // yaw (slightly higher - fused with gyro)
    odom.twist.covariance[0] = 1e-3;  // vx
    odom.twist.covariance[35] = 1e-3; // vyaw

    // ========== PUBLISH IMU MESSAGE ==========
    // Republish IMU data for EKF fusion (orientation = integrated yaw from above)
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "imu_link";

    // Orientation: use fused yaw (roll=pitch=0 for differential drive)
    imu_msg.orientation = odom.pose.pose.orientation;

    // Angular velocity (raw gyro data)
    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;

    // Linear acceleration
    imu_msg.linear_acceleration.x = acc_x;
    imu_msg.linear_acceleration.y = acc_y;
    imu_msg.linear_acceleration.z = acc_z;

    // Covariance (trust gyro_z for yaw, accel less reliable)
    for (int i = 0; i < 9; ++i) {
      imu_msg.orientation_covariance[i] = 0.0;
      imu_msg.angular_velocity_covariance[i] = 0.0;
      imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    imu_msg.orientation_covariance[8] = 0.01;  // yaw variance
    imu_msg.angular_velocity_covariance[8] = 0.001;  // gyro_z variance
    imu_msg.linear_acceleration_covariance[0] = 0.1;  // acc_x
    imu_msg.linear_acceleration_covariance[4] = 0.1;  // acc_y
    imu_msg.linear_acceleration_covariance[8] = 0.1;  // acc_z

    // Throttle publishing to prevent overload (STM32 sends >100Hz, we publish at target rate)
    // Use nanoseconds for precise timing to avoid accumulation errors
    auto time_since_last_pub = now - last_pub_time_;
    if (time_since_last_pub.nanoseconds() >= odom_publish_period_.nanoseconds()) {
      odom_pub_->publish(odom);
      imu_pub_->publish(imu_msg);
      // Update to next target time instead of "now" to maintain consistent rate
      last_pub_time_ = last_pub_time_ + odom_publish_period_;
      
      // Safety: if we're falling behind, reset to current time
      if ((now - last_pub_time_).nanoseconds() > odom_publish_period_.nanoseconds()) {
        last_pub_time_ = now;
      }
    }

    // Update last integration time (always update for accurate dt calculation)
    last_time_ = now;
  }

  // parameters/state
  double wheel_radius_{0.05};
  double wheel_separation_{0.46};

  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double last_gyro_z_{0.0};  // For gyro smoothing
  rclcpp::Time last_time_;
  rclcpp::Time last_pub_time_;
  rclcpp::Duration odom_publish_period_{rclcpp::Duration::from_seconds(0.0)};

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sensor_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorOdomNode>());
  rclcpp::shutdown();
  return 0;
}