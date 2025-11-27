#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

using std::placeholders::_1;

class MotorOdomNode : public rclcpp::Node {
public:
  MotorOdomNode() : Node("stm32_odom") {
    // === PARAMETERS ===
    this->declare_parameter<double>("wheel_radius", 0.05);      // Đã cập nhật
    this->declare_parameter<double>("wheel_separation", 0.42);  // Đã cập nhật từ 0.46 -> 0.42
    this->declare_parameter<double>("odom_publish_rate", 50.0);
    
    // Topics
    this->declare_parameter<std::string>("sensor_data_topic", "/sensor_data");
    this->declare_parameter<std::string>("odom_topic", "/diff_cont/odom");
    this->declare_parameter<std::string>("imu_topic", "/imu");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_footprint");

    // Get Params
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();

    double pub_rate = this->get_parameter("odom_publish_rate").as_double();
    odom_publish_period_ = rclcpp::Duration::from_seconds(1.0 / pub_rate);
    last_pub_time_ = this->now();

    // Topics
    std::string sensor_topic = this->get_parameter("sensor_data_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    // Subscribers & Publishers
    sensor_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      sensor_topic, 10, std::bind(&MotorOdomNode::sensorDataCallback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    last_time_ = this->now();
    resetOdometry();

    RCLCPP_INFO(this->get_logger(), "✅ STM32 Odom Node Started. Radius: %.3f, Base: %.3f", 
                wheel_radius_, wheel_separation_);
  }

private:
  void resetOdometry() {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
  }

  void sensorDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    rclcpp::Time now = this->now();

    // Validate data format
    if (msg->data.size() < 8) return;

    // 1. Extract Data
    // [gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, speed_left, speed_right]
    double gyro_x = msg->data[0];
    double gyro_y = msg->data[1];
    double gyro_z = msg->data[2]; // rad/s
    
    double acc_x = msg->data[3];  // m/s^2
    double acc_y = msg->data[4];
    double acc_z = msg->data[5];

    double v_left = msg->data[6];  // m/s
    double v_right = msg->data[7]; // m/s

    // Calculate dt
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {
      last_time_ = now;
      return;
    }

    // ========== 2. ODOMETRY CALCULATION (WITH ZUPT FIX) ==========
    
    double vx = 0.0;
    double vtheta_fused = 0.0;

    // Check if robot is stationary (Zero-Velocity Update)
    // Threshold: 0.005 m/s (5mm/s)
    bool is_stationary = (std::abs(v_right) < 0.005) && (std::abs(v_left) < 0.005);

    if (is_stationary) {
        // --- ROBOT ĐỨNG YÊN ---
        // Ép mọi vận tốc về 0 để cắt đứt nhiễu trôi (Drift)
        vx = 0.0;
        vtheta_fused = 0.0; 
        
        // (Tùy chọn) Có thể reset gyro bias ở đây nếu muốn dynamic calibration
    } else {
        // --- ROBOT DI CHUYỂN ---
        // 1. Tính vận tốc dài từ trung bình 2 bánh
        vx = (v_right + v_left) / 2.0;
        
        // 2. Tính vận tốc góc từ Encoder
        double vtheta_encoder = (v_right - v_left) / wheel_separation_;

        // 3. Sensor Fusion: Kết hợp Gyro và Encoder
        // alpha = 0.8: Tin tưởng Gyro 80% (vì Gyro nhạy hơn khi quay)
        const double alpha = 0.98;
        vtheta_fused = alpha * gyro_z + (1.0 - alpha) * vtheta_encoder;
    }

    // ========== 3. INTEGRATION (Tích phân vị trí) ==========
    double d_yaw = vtheta_fused * dt;
    double dx = vx * std::cos(yaw_) * dt; // Simple Euler integration
    double dy = vx * std::sin(yaw_) * dt;

    yaw_ += d_yaw;
    x_ += dx;
    y_ += dy;

    // Normalize Yaw [-PI, PI]
    while (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
    while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

    // ========== 4. PUBLISH ==========
    
    // Chỉ publish theo tần số định sẵn (ví dụ 50Hz)
    if ((now - last_pub_time_) >= odom_publish_period_) {
        
        // --- A. Publish Odometry ---
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        // Pose
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // Twist (Velocity)
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = vtheta_fused;

        // Covariance (Cần thiết cho EKF)
        // Pose covariance
        odom.pose.covariance[0] = 0.001; // x
        odom.pose.covariance[7] = 0.001; // y
        odom.pose.covariance[35] = 0.001; // yaw
        // Twist covariance
        odom.twist.covariance[0] = 0.001; // vx
        odom.twist.covariance[35] = 0.001; // vyaw

        odom_pub_->publish(odom);

        // --- B. Publish TF (Odom -> Base) ---
        // EKF sẽ publish map->odom, node này publish odom->base
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;
        
        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = odom.pose.pose.orientation;
        
        tf_broadcaster_->sendTransform(tf_msg);

        // --- C. Publish IMU (For EKF) ---
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = "imu_link"; // Đảm bảo trùng với URDF

        // Orientation (Lấy từ Odometry đã tính toán)
        imu_msg.orientation = odom.pose.pose.orientation;
        
        // Angular Velocity (Raw Gyro)
        imu_msg.angular_velocity.x = gyro_x;
        imu_msg.angular_velocity.y = gyro_y;
        imu_msg.angular_velocity.z = gyro_z;

        // Linear Acceleration (Raw Accel)
        imu_msg.linear_acceleration.x = acc_x;
        imu_msg.linear_acceleration.y = acc_y;
        imu_msg.linear_acceleration.z = acc_z;
        
        // Covariance
        imu_msg.orientation_covariance[8] = 0.001;
        imu_msg.angular_velocity_covariance[8] = 0.001;
        imu_msg.linear_acceleration_covariance[0] = 0.01;

        imu_pub_->publish(imu_msg);

        last_pub_time_ = now;
    }

    last_time_ = now;
  }

  // Variables
  double wheel_radius_;
  double wheel_separation_;
  double x_, y_, yaw_;
  
  rclcpp::Time last_time_;
  rclcpp::Time last_pub_time_;
  rclcpp::Duration odom_publish_period_{rclcpp::Duration::from_seconds(0.0)};

  std::string odom_frame_, base_frame_;

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