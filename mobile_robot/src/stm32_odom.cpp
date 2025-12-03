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
    this->declare_parameter<double>("wheel_radius", 0.05);      
    this->declare_parameter<double>("wheel_separation", 0.42);  
    this->declare_parameter<double>("odom_publish_rate", 50.0);
    
    // 🔥 NEW PARAMETER: GYRO SCALE FACTOR
    // Đã tính toán từ thực nghiệm: 90 / 86 = 1.046. Nhân với hệ số cũ 1.274 -> 1.333
    this->declare_parameter<double>("gyro_scale_factor", 1.07); 

    // Topics
    this->declare_parameter<std::string>("sensor_data_topic", "/sensor_data");
    this->declare_parameter<std::string>("odom_topic", "/diff_cont/odom");
    this->declare_parameter<std::string>("imu_topic", "/imu");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_footprint");

    // Get Params
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    gyro_scale_factor_ = this->get_parameter("gyro_scale_factor").as_double();

    double pub_rate = this->get_parameter("odom_publish_rate").as_double();
    odom_publish_period_ = rclcpp::Duration::from_seconds(1.0 / pub_rate);
    last_pub_time_ = this->now();

    // Strings
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

    RCLCPP_INFO(this->get_logger(), "✅ STM32 Odom Ready. Scale Factor: %.3f", gyro_scale_factor_);
  }

private:
  void resetOdometry() {
    x_ = 0.0; y_ = 0.0; yaw_ = 0.0;
  }

  void sensorDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    rclcpp::Time now = this->now();
    if (msg->data.size() < 8) return;

    // 1. Extract Data
    double gyro_x = msg->data[0];
    double gyro_y = msg->data[1];
    double gyro_z = msg->data[2]; 
    
    double acc_x = msg->data[3];
    double acc_y = msg->data[4];
    double acc_z = msg->data[5];

    double v_left = msg->data[6];
    double v_right = msg->data[7];

    double dt = (now - last_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) { last_time_ = now; return; }

    // 2. Logic Calculation
    double vx = 0.0;
    double vtheta_fused = 0.0;
    bool is_stationary = (std::abs(v_right) < 0.005) && (std::abs(v_left) < 0.005);

    if (is_stationary) {
        vx = 0.0;
        vtheta_fused = 0.0; 
        // Zero IMU drift when stationary
        gyro_z = 0.0; gyro_x = 0.0; gyro_y = 0.0;
    } else {
        vx = (v_right + v_left) / 2.0;
        
        // 🔥 APPLY SCALE FACTOR HERE
        // Nhân hệ số bù để sửa lỗi thiếu góc (Under-steering)
        double gyro_z_corrected = gyro_z * gyro_scale_factor_;
        
        // Use 100% Gyro
        vtheta_fused = gyro_z_corrected; 
        
        // Update raw gyro variable for IMU publishing too (consistency)
        gyro_z = gyro_z_corrected;
    }

    // 3. Integration
    double d_yaw = vtheta_fused * dt;
    double dx = vx * std::cos(yaw_) * dt;
    double dy = vx * std::sin(yaw_) * dt;

    yaw_ += d_yaw;
    x_ += dx;
    y_ += dy;

    // Normalize Yaw
    while (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
    while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

    // 4. Publish (Throttled)
    if ((now - last_pub_time_) >= odom_publish_period_) {
        // --- Odom Msg ---
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = vtheta_fused;

        // Covariance - Rất thấp vì đã scale gyro chính xác
        odom.pose.covariance[0] = 0.0001;  // x position variance
        odom.pose.covariance[7] = 0.0001;  // y position variance  
        odom.pose.covariance[35] = 0.0001; // yaw variance
        odom.twist.covariance[0] = 0.0001; // vx variance
        odom.twist.covariance[35] = 0.0001; // vyaw variance

        odom_pub_->publish(odom);

        // --- TF ---
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);

        // --- IMU Msg ---
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = "imu_link"; 
        imu_msg.orientation = odom.pose.pose.orientation;
        imu_msg.angular_velocity.x = gyro_x;
        imu_msg.angular_velocity.y = gyro_y;
        imu_msg.angular_velocity.z = gyro_z; // Scaled gyro
        imu_msg.linear_acceleration.x = acc_x;
        imu_msg.linear_acceleration.y = acc_y;
        imu_msg.linear_acceleration.z = acc_z;
        
        imu_msg.orientation_covariance[8] = 0.001;
        imu_msg.angular_velocity_covariance[8] = 0.001;
        imu_msg.linear_acceleration_covariance[0] = 0.01;

        imu_pub_->publish(imu_msg);

        last_pub_time_ = now;
    }
    last_time_ = now;
  }

  // Members
  double wheel_radius_, wheel_separation_, gyro_scale_factor_;
  double x_, y_, yaw_;
  rclcpp::Time last_time_, last_pub_time_;
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