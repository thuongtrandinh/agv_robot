/**
 * @file aruco_map_localizer.cpp
 * @brief ArUco-based absolute localization using pre-defined marker positions
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

struct MarkerInfo {
  int id;
  double x, y, z;
  double roll, pitch, yaw;
  double size;
};

class ArucoMapLocalizer : public rclcpp::Node {
public:
  ArucoMapLocalizer() : Node("aruco_map_localizer"),
    dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250))
  {
    // Parameters
    auto image_topic = declare_parameter<std::string>("image_topic", "/zed2/left/image_raw");
    auto camera_info_topic = declare_parameter<std::string>("camera_info_topic", "/zed2/left/camera_info");
    auto config_file = declare_parameter<std::string>("marker_config", "");

    // Default: load the marker YAML saved by mapping node
    if (config_file.empty()) {
      auto pkg_dir = ament_index_cpp::get_package_share_directory("agv_mapping_with_knowns_poses");
      config_file = pkg_dir + "/maps/aruco_markers/aruco_map_positions.yaml";
    }

    loadMarkerConfig(config_file);

    // Publishers
    pub_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/aruco/pose_with_covariance", 10);
    pub_debug_image_ = create_publisher<sensor_msgs::msg::Image>("/aruco/debug_image", 5);

    // Subscribers
    auto qos = rclcpp::SensorDataQoS();
    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, qos,
      std::bind(&ArucoMapLocalizer::cameraInfoCallback, this, std::placeholders::_1));

    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos,
      std::bind(&ArucoMapLocalizer::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "✅ ArUco Map Localizer started");
    RCLCPP_INFO(get_logger(), "   Loaded %zu markers", markers_.size());
  }

private:

  // ================================
  // LOAD MARKER POSITIONS FROM YAML
  // ================================
  void loadMarkerConfig(const std::string &config_file) {
    RCLCPP_INFO(get_logger(), "📖 Loading marker config: %s", config_file.c_str());

    try {
      YAML::Node config = YAML::LoadFile(config_file);

      // Load covariance parameters (optional)
      if (config["covariance"]) {
        pos_variance_ = config["covariance"]["position_variance"].as<double>(0.0001);
        ori_variance_ = config["covariance"]["orientation_variance"].as<double>(0.001);
        dist_scaling_ = config["covariance"]["distance_scaling"].as<double>(0.00005);
      }

      // Load markers
      auto markers = config["markers"];
      for (auto it : markers) {
        auto node = it.second;
        MarkerInfo info;

        info.id = node["id"].as<int>();
        info.x = node["position"]["x"].as<double>();
        info.y = node["position"]["y"].as<double>();
        info.z = node["position"]["z"].as<double>();
        info.roll = node["orientation"]["roll"].as<double>();
        info.pitch = node["orientation"]["pitch"].as<double>();
        info.yaw = node["orientation"]["yaw"].as<double>();
        info.size = node["size"].as<double>(0.173);

        markers_[info.id] = info;

        RCLCPP_INFO(get_logger(), "   Marker %d loaded (%.2f, %.2f)",
                    info.id, info.x, info.y);
      }

    } catch (std::exception &e) {
      RCLCPP_ERROR(get_logger(), "❌ Failed to load marker file: %s", e.what());
    }
  }

  // ======================
  // CAMERA INFO CALLBACK
  // ======================
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (has_cam_info_) return;

    K_ = (cv::Mat_<double>(3,3) <<
      msg->k[0], msg->k[1], msg->k[2],
      msg->k[3], msg->k[4], msg->k[5],
      msg->k[6], msg->k[7], msg->k[8]);

    dist_coeffs_ = cv::Mat(1, 5, CV_64F);
    for (int i = 0; i < 5; i++)
      dist_coeffs_.at<double>(0, i) = msg->d[i];

    camera_frame_ = msg->header.frame_id;
    has_cam_info_ = true;

    RCLCPP_INFO(get_logger(), "📸 Camera info OK");
  }

  // ======================
  // IMAGE CALLBACK
  // ======================
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!has_cam_info_) return;

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "cv_bridge failed");
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids);

    if (!ids.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Detected %zu markers", ids.size());
    }

    for (size_t i = 0; i < ids.size(); i++) {
      int id = ids[i];

      if (markers_.count(id) == 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Marker %d not in map!", id);
        continue;
      }

      std::vector<cv::Point3f> obj_points = {
        {-markers_[id].size/2,  markers_[id].size/2, 0},
        { markers_[id].size/2,  markers_[id].size/2, 0},
        { markers_[id].size/2, -markers_[id].size/2, 0},
        {-markers_[id].size/2, -markers_[id].size/2, 0}
      };

      cv::Vec3d rvec, tvec;
      cv::solvePnP(obj_points, corners[i], K_, dist_coeffs_, rvec, tvec);

      calculateRobotPose(id, rvec, tvec, msg->header.stamp);
    }

    pub_debug_image_->publish(*cv_ptr->toImageMsg());
  }

  // ==========================
  // COMPUTE ROBOT POSE IN MAP
  // ==========================
  void calculateRobotPose(int id, const cv::Vec3d &rvec,
                          const cv::Vec3d &tvec,
                          const builtin_interfaces::msg::Time &stamp)
  {
    const MarkerInfo &mk = markers_[id];

    // Camera -> Marker transform
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);

    tf2::Matrix3x3 rot(
      R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
      R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
      R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2)
    );

    tf2::Transform T_cam_marker(rot, tf2::Vector3(tvec[0], tvec[1], tvec[2]));

    // Marker in map transform
    tf2::Quaternion q_mk;
    q_mk.setRPY(mk.roll, mk.pitch, mk.yaw);

    tf2::Transform T_map_marker(q_mk, tf2::Vector3(mk.x, mk.y, mk.z));

    // Compute robot pose
    tf2::Transform T_map_base = T_map_marker * T_cam_marker.inverse();

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = "map";

    out.pose.pose.position.x = T_map_base.getOrigin().x();
    out.pose.pose.position.y = T_map_base.getOrigin().y();
    out.pose.pose.position.z = T_map_base.getOrigin().z();

    auto q = T_map_base.getRotation();
    out.pose.pose.orientation = tf2::toMsg(q);

    // Covariance
    double dist = std::sqrt(tvec[0]*tvec[0] + tvec[1]*tvec[1] + tvec[2]*tvec[2]);
    double var_pos = pos_variance_ + dist * dist_scaling_;
    double var_ori = ori_variance_ + dist * dist_scaling_;

    out.pose.covariance[0]  = var_pos;
    out.pose.covariance[7]  = var_pos;
    out.pose.covariance[35] = var_ori;

    pub_pose_cov_->publish(out);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "📍 Marker %d → Robot at (%.2f, %.2f)",
                         id,
                         out.pose.pose.position.x,
                         out.pose.pose.position.y);
  }

  // Members
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  std::map<int, MarkerInfo> markers_;

  cv::Mat K_, dist_coeffs_;
  bool has_cam_info_ = false;
  std::string camera_frame_;

  double pos_variance_ = 0.0001;
  double ori_variance_ = 0.001;
  double dist_scaling_ = 0.00005;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoMapLocalizer>());
  rclcpp::shutdown();
  return 0;
}
