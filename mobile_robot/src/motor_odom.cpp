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
  MotorOdomNode() : Node("motor_odom") {
    // Parameters (tune as needed)
    this->declare_parameter<double>("wheel_radius", 0.05);
    this->declare_parameter<double>("wheel_separation", 0.46);
    this->declare_parameter<int>("left_index", 1);
    this->declare_parameter<int>("right_index", 0);
    this->declare_parameter<bool>("feedback_is_linear_velocity", true);
    // New param: odom publish rate (Hz)
    this->declare_parameter<double>("odom_publish_rate", 13.0);
    // Topics and frames
    this->declare_parameter<std::string>("imu_topic", "/imu");
    this->declare_parameter<std::string>("motor_topic", "/motor_feedback");
    this->declare_parameter<std::string>("odom_topic", "/diff_cont/odom");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_footprint");

    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    left_idx_ = this->get_parameter("left_index").as_int();
    right_idx_ = this->get_parameter("right_index").as_int();
    feedback_linear_ = this->get_parameter("feedback_is_linear_velocity").as_bool();

    double pub_rate = this->get_parameter("odom_publish_rate").as_double();
    if (pub_rate <= 0.0) pub_rate = 13.0;
    odom_publish_period_ = rclcpp::Duration::from_seconds(1.0 / pub_rate);
    last_pub_time_ = this->now() - odom_publish_period_;

    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string motor_topic = this->get_parameter("motor_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 10, std::bind(&MotorOdomNode::imuCallback, this, _1));

    motor_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      motor_topic, 10, std::bind(&MotorOdomNode::motorCallback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    last_time_ = this->now();

    // Reset odometry state on node startup
    resetOdometry();

    RCLCPP_INFO(this->get_logger(),
                "motor_odom started. motor_topic: %s  imu_topic: %s  (motor_feedback format: [right, left], left_index=%d right_index=%d)",
                motor_topic.c_str(), imu_topic.c_str(), left_idx_, right_idx_);
  }

private:
  void resetOdometry() {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    last_imu_q_ = geometry_msgs::msg::Quaternion();
    RCLCPP_INFO(this->get_logger(), "Odometry state reset to initial values.");
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    last_imu_q_ = msg->orientation;
  }

  void motorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    rclcpp::Time now = this->now();

    if (msg->data.size() <= std::max(left_idx_, right_idx_)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "motor_feedback size %zu too small", msg->data.size());
      return;
    }

    double left_val = msg->data[left_idx_];
    double right_val = msg->data[right_idx_];

    // Interpret feedback
    double v_left = 0.0, v_right = 0.0;
    if (feedback_linear_) {
      // feedback already linear velocity (m/s)
      v_left = left_val;
      v_right = right_val;
    } else {
      // feedback angular velocity (rad/s) -> convert to linear
      v_left = left_val * wheel_radius_;
      v_right = right_val * wheel_radius_;
    }

    // Compute robot linear & angular velocities
    double vx = (v_right + v_left) / 2.0;
    double vtheta = (v_right - v_left) / wheel_separation_;

    // compute dt and integrate pose (simple Euler)
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) {
      last_time_ = now;
      return;
    }

    double dx = vx * dt;
    double dtheta = vtheta * dt;

    yaw_ += dtheta;
    x_ += dx * std::cos(yaw_);
    y_ += dx * std::sin(yaw_);

    // Build odom message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    if (validQuaternion(last_imu_q_)) {
      odom.pose.pose.orientation = last_imu_q_;
      // Update yaw_ from IMU orientation for integration consistency
      tf2::Quaternion q(last_imu_q_.x, last_imu_q_.y, last_imu_q_.z, last_imu_q_.w);
      double roll, pitch, imu_yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, imu_yaw);
      yaw_ = imu_yaw;
    } else {
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw_);
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
    }

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vtheta;

    // Optionally fill covariance (small defaults)
    for (int i = 0; i < 36; ++i) { odom.pose.covariance[i] = 0.0; odom.twist.covariance[i] = 0.0; }
    odom.pose.covariance[0] = 1e-3;   // x
    odom.pose.covariance[7] = 1e-3;   // y
    odom.pose.covariance[35] = 1e-2;  // yaw
    odom.twist.covariance[0] = 1e-3;  // vx
    odom.twist.covariance[35] = 1e-3; // vyaw

    // Publish only if not throttled (odom_publish_period_ == 0 => no throttling)
    if (odom_publish_period_.seconds() <= 0.0 || (now - last_pub_time_) >= odom_publish_period_) {
      odom_pub_->publish(odom);

      // Broadcast TF odom -> base
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now;
      t.header.frame_id = odom_frame_;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = x_;
      t.transform.translation.y = y_;
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(t);

      // update last publish time
      last_pub_time_ = now;
    }

    // update last integration time
    last_time_ = now;
  }

  bool validQuaternion(const geometry_msgs::msg::Quaternion &q) {
    return !(q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0);
  }

  // parameters/state
  double wheel_radius_{0.05};
  double wheel_separation_{0.46};
  int left_idx_{1}, right_idx_{0};  // default mapping: motor_feedback[0]=right, [1]=left
  bool feedback_linear_{true};

  double x_{0.0}, y_{0.0}, yaw_{0.0};
  rclcpp::Time last_time_;
  rclcpp::Time last_pub_time_;
  rclcpp::Duration odom_publish_period_{rclcpp::Duration::from_seconds(0.0)};
  geometry_msgs::msg::Quaternion last_imu_q_{};

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorOdomNode>());
  rclcpp::shutdown();
  return 0;
}