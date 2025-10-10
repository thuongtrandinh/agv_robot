#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// TF2
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class SimArucoNode : public rclcpp::Node {
public:
  SimArucoNode()
  : Node("aruco_detector"),
    dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250))
  {
    // Parameters
    image_topic_        = this->declare_parameter<std::string>("image_topic", "/zed2/left/image_raw");
    camera_info_topic_  = this->declare_parameter<std::string>("camera_info_topic", "/zed2/left/camera_info");
    marker_size_        = this->declare_parameter<double>("marker_size", 0.173); // mét
    cam_frame_id_       = this->declare_parameter<std::string>("camera_frame_id", "zed2_left_camera_frame");

    // Publishers
    pub_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("aruco/poses", 10);
    pub_pose_       = this->create_publisher<geometry_msgs::msg::PoseStamped>("aruco/pose", 10);
    pub_image_      = this->create_publisher<sensor_msgs::msg::Image>("aruco/image", 5);
    pub_ids_        = this->create_publisher<std_msgs::msg::Int32MultiArray>("aruco/ids", 10);
    pub_info_       = this->create_publisher<std_msgs::msg::Float32MultiArray>("aruco/info", 10);
    pub_markers_    = this->create_publisher<visualization_msgs::msg::MarkerArray>("aruco/markers", 10);

    // TF
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribers (sensor QoS)
    auto sensor_qos = rclcpp::SensorDataQoS();
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos,
      std::bind(&SimArucoNode::cameraInfoCallback, this, std::placeholders::_1));

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, sensor_qos,
      std::bind(&SimArucoNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "✅ ArUco detector started.\n   Image: %s\n   CameraInfo: %s\n   Marker size: %.3f m",
      image_topic_.c_str(), camera_info_topic_.c_str(), marker_size_);
  }

private:
  // Receive intrinsics
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (has_cam_info_) return; // lấy một lần là đủ

    K_ = (cv::Mat_<double>(3,3) <<
      msg->k[0], msg->k[1], msg->k[2],
      msg->k[3], msg->k[4], msg->k[5],
      msg->k[6], msg->k[7], msg->k[8]);

    if (!msg->d.empty()) {
      dist_coeffs_ = cv::Mat(msg->d).clone().reshape(1,1);
    } else {
      dist_coeffs_ = cv::Mat::zeros(1,5,CV_64F);
    }

    cam_frame_id_ = msg->header.frame_id.empty() ? cam_frame_id_ : msg->header.frame_id;

    has_cam_info_ = true;
    RCLCPP_INFO(this->get_logger(), "📷 Camera info received (frame_id: %s).", cam_frame_id_.c_str());
  }

  // Robust image conversion: rgb8 / bgr8 / rgba8 fallback
  bool toBGR(const sensor_msgs::msg::Image & msg, cv::Mat & out_bgr)
  {
    try {
      // Preferred: ask cv_bridge to convert to BGR directly
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      out_bgr = cv_ptr->image;
      return true;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "cv_bridge direct bgr8 convert failed: %s. Trying fallback...", e.what());
    }

    try {
      if (msg.encoding == "rgb8") {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
        cv::cvtColor(cv_ptr->image, out_bgr, cv::COLOR_RGB2BGR);
        return true;
      } else if (msg.encoding == "bgr8") {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        out_bgr = cv_ptr->image;
        return true;
      } else if (msg.encoding == "rgba8") {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "rgba8");
        cv::cvtColor(cv_ptr->image, out_bgr, cv::COLOR_RGBA2BGR);
        return true;
      } else if (msg.encoding == "bgra8") {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgra8");
        cv::cvtColor(cv_ptr->image, out_bgr, cv::COLOR_BGRA2BGR);
        return true;
      } else {
        // last resort: raw buffer reshape (assume rgb8)
        const size_t expected = static_cast<size_t>(msg.height) * msg.width * 3;
        if (msg.data.size() == expected) {
          cv::Mat rgb(msg.height, msg.width, CV_8UC3, const_cast<uint8_t*>(msg.data.data()));
          cv::cvtColor(rgb, out_bgr, cv::COLOR_RGB2BGR);
          return true;
        }
        RCLCPP_ERROR(this->get_logger(),
          "Unsupported encoding '%s' (data size=%zu).",
          msg.encoding.c_str(), msg.data.size());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Fallback convert failed: %s", e.what());
    }
    return false;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!has_cam_info_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "⏳ Waiting for /camera_info before processing images...");
      return;
    }

    // Convert to BGR
    cv::Mat frame_bgr;
    if (!toBGR(*msg, frame_bgr)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "❌ Failed to convert Image to BGR.");
      return;
    }

    // Detect ArUco
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame_bgr, dictionary_, corners, ids);

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = msg->header.stamp;
    pose_array.header.frame_id = cam_frame_id_;

    std_msgs::msg::Int32MultiArray ids_msg;
    std_msgs::msg::Float32MultiArray info_msg;
    visualization_msgs::msg::MarkerArray marker_array;

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(frame_bgr, corners, ids);

      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, K_, dist_coeffs_, rvecs, tvecs);

      for (size_t i = 0; i < ids.size(); ++i) {
        // vẽ trục
        cv::aruco::drawAxis(frame_bgr, K_, dist_coeffs_, rvecs[i], tvecs[i], marker_size_*0.4);

        geometry_msgs::msg::Pose pose;
        pose.position.x = tvecs[i][0];
        pose.position.y = tvecs[i][1];
        pose.position.z = tvecs[i][2];

        // Rodrigues -> quaternion
        cv::Mat R;
        cv::Rodrigues(rvecs[i], R);
        double qx, qy, qz, qw;
        rotmToQuat(R, qx, qy, qz, qw);
        pose.orientation.x = qx;
        pose.orientation.y = qy;
        pose.orientation.z = qz;
        pose.orientation.w = qw;

        pose_array.poses.push_back(pose);
        ids_msg.data.push_back(ids[i]);

        // distance (chuẩn Euclid từ tvec)
        const double dist = std::sqrt(
          tvecs[i][0]*tvecs[i][0] + tvecs[i][1]*tvecs[i][1] + tvecs[i][2]*tvecs[i][2]);
        info_msg.data.push_back(static_cast<float>(ids[i]));
        info_msg.data.push_back(static_cast<float>(dist));

        // publish PoseStamped của marker đầu tiên
        if (i == 0) {
          geometry_msgs::msg::PoseStamped ps;
          ps.header = pose_array.header;
          ps.pose = pose;
          pub_pose_->publish(ps);
        }

        // Marker RViz
        visualization_msgs::msg::Marker marker;
        marker.header = pose_array.header;
        marker.ns = "aruco";
        marker.id = ids[i];
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = marker_size_;
        marker.scale.y = marker_size_;
        marker.scale.z = 0.01;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.7f;
        marker_array.markers.push_back(marker);

        // TF
        geometry_msgs::msg::TransformStamped t;
        t.header = pose_array.header;
        t.child_frame_id = "aruco_marker_" + std::to_string(ids[i]);
        t.transform.translation.x = pose.position.x;
        t.transform.translation.y = pose.position.y;
        t.transform.translation.z = pose.position.z;
        t.transform.rotation = pose.orientation;
        tf_broadcaster_->sendTransform(t);

        // Overlay text
        // Lấy tâm marker để vẽ chữ gần đó
        const int cx = static_cast<int>((corners[i][0].x + corners[i][2].x) * 0.5);
        const int cy = static_cast<int>((corners[i][0].y + corners[i][2].y) * 0.5);
        cv::putText(frame_bgr,
          "ID=" + std::to_string(ids[i]) + " Dist=" + std::to_string(dist*100.0) + "cm",
          cv::Point(cx - 60, cy - 20),
          cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,255), 2);
      }

      pub_pose_array_->publish(pose_array);
      pub_ids_->publish(ids_msg);
      pub_info_->publish(info_msg);
      pub_markers_->publish(marker_array);
    }

    // Publish debug image (nếu có subscriber)
    if (pub_image_->get_subscription_count() > 0) {
      auto img_msg = cv_bridge::CvImage(msg->header, "bgr8", frame_bgr).toImageMsg();
      pub_image_->publish(*img_msg);
    }
  }

  static void rotmToQuat(const cv::Mat &R, double &qx, double &qy, double &qz, double &qw) {
    const double tr = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    if (tr > 0) {
      double S = std::sqrt(tr + 1.0) * 2.0;
      qw = 0.25 * S;
      qx = (R.at<double>(2,1) - R.at<double>(1,2)) / S;
      qy = (R.at<double>(0,2) - R.at<double>(2,0)) / S;
      qz = (R.at<double>(1,0) - R.at<double>(0,1)) / S;
    } else if (R.at<double>(0,0) > R.at<double>(1,1) && R.at<double>(0,0) > R.at<double>(2,2)) {
      double S = std::sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2)) * 2.0;
      qw = (R.at<double>(2,1) - R.at<double>(1,2)) / S;
      qx = 0.25 * S;
      qy = (R.at<double>(0,1) + R.at<double>(1,0)) / S;
      qz = (R.at<double>(0,2) + R.at<double>(2,0)) / S;
    } else if (R.at<double>(1,1) > R.at<double>(2,2)) {
      double S = std::sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2)) * 2.0;
      qw = (R.at<double>(0,2) - R.at<double>(2,0)) / S;
      qx = (R.at<double>(0,1) + R.at<double>(1,0)) / S;
      qy = 0.25 * S;
      qz = (R.at<double>(1,2) + R.at<double>(2,1)) / S;
    } else {
      double S = std::sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1)) * 2.0;
      qw = (R.at<double>(1,0) - R.at<double>(0,1)) / S;
      qx = (R.at<double>(0,2) + R.at<double>(2,0)) / S;
      qy = (R.at<double>(1,2) + R.at<double>(2,1)) / S;
      qz = 0.25 * S;
    }
  }

  // Members
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string cam_frame_id_;

  double marker_size_{0.173};

  bool has_cam_info_{false};
  cv::Mat K_ = cv::Mat::eye(3,3,CV_64F);
  cv::Mat dist_coeffs_ = cv::Mat::zeros(1,5,CV_64F);
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_array_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_ids_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_info_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimArucoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
