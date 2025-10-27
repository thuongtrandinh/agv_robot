/**
 * @file aruco_map_localizer.cpp
 * @brief ArUco-based absolute localization using pre-defined marker positions
 * 
 * This node uses ArUco markers with KNOWN positions in the map frame to provide
 * ABSOLUTE robot localization (ground truth when markers are detected).
 * 
 * Key concept: 
 * - Markers have FIXED, KNOWN positions in map → Ground truth
 * - When camera detects marker → Calculate exact robot pose in map
 * - Publish with VERY LOW covariance → EKF trusts absolutely
 * 
 * Author: Thuong Tran Dinh
 * Date: October 27, 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
  double x, y, z;           // Position in map frame
  double roll, pitch, yaw;  // Orientation in map frame  
  double size;              // Marker size in meters
};

class ArucoMapLocalizer : public rclcpp::Node {
public:
  ArucoMapLocalizer() : Node("aruco_map_localizer"),
    dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250))
  {
    // Parameters
    auto image_topic = this->declare_parameter<std::string>("image_topic", "/zed2/left/image_raw");
    auto camera_info_topic = this->declare_parameter<std::string>("camera_info_topic", "/zed2/left/camera_info");
    auto config_file = this->declare_parameter<std::string>("marker_config", "");
    
    // Load marker positions from config
    if (config_file.empty()) {
      auto pkg_dir = ament_index_cpp::get_package_share_directory("agv_localization");
      config_file = pkg_dir + "/config/aruco_map_positions.yaml";
    }
    loadMarkerConfig(config_file);
    
    // Publishers
    pub_pose_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "aruco/pose_with_covariance", 10);
    pub_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "aruco/detected_markers", 10);
    pub_debug_image_ = this->create_publisher<sensor_msgs::msg::Image>(
      "aruco/debug_image", 5);
      
    // Subscribers
    auto qos = rclcpp::SensorDataQoS();
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, qos,
      std::bind(&ArucoMapLocalizer::cameraInfoCallback, this, std::placeholders::_1));
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos,
      std::bind(&ArucoMapLocalizer::imageCallback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "✅ ArUco Map Localizer started");
    RCLCPP_INFO(this->get_logger(), "   Loaded %zu markers from config", markers_.size());
    RCLCPP_INFO(this->get_logger(), "   Covariance: position=%.6f, orientation=%.6f",
                pos_variance_, ori_variance_);
  }

private:
  void loadMarkerConfig(const std::string& config_file) {
    RCLCPP_INFO(this->get_logger(), "📖 Loading marker config from: %s", config_file.c_str());
    
    try {
      YAML::Node config = YAML::LoadFile(config_file);
      
      // Load covariance settings
      if (config["covariance"]) {
        pos_variance_ = config["covariance"]["position_variance"].as<double>(0.0001);
        ori_variance_ = config["covariance"]["orientation_variance"].as<double>(0.001);
        dist_scaling_ = config["covariance"]["distance_scaling"].as<double>(0.00005);
      }
      
      // Load marker positions
      auto markers = config["aruco_markers"];
      for (auto it = markers.begin(); it != markers.end(); ++it) {
        auto marker_node = it->second;
        MarkerInfo info;
        info.id = marker_node["id"].as<int>();
        info.x = marker_node["position"]["x"].as<double>();
        info.y = marker_node["position"]["y"].as<double>();
        info.z = marker_node["position"]["z"].as<double>();
        info.roll = marker_node["orientation"]["roll"].as<double>();
        info.pitch = marker_node["orientation"]["pitch"].as<double>();
        info.yaw = marker_node["orientation"]["yaw"].as<double>();
        info.size = marker_node["size"].as<double>(0.173);
        
        markers_[info.id] = info;
        RCLCPP_INFO(this->get_logger(), "   Marker %d: (%.2f, %.2f, %.2f)", 
                    info.id, info.x, info.y, info.z);
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to load config: %s", e.what());
    }
  }
  
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (has_cam_info_) return;
    
    K_ = (cv::Mat_<double>(3,3) <<
      msg->k[0], msg->k[1], msg->k[2],
      msg->k[3], msg->k[4], msg->k[5],
      msg->k[6], msg->k[7], msg->k[8]);
    
    dist_coeffs_ = cv::Mat(1, 5, CV_64F);
    for (int i = 0; i < 5; i++) {
      dist_coeffs_.at<double>(0, i) = msg->d[i];
    }
    
    camera_frame_ = msg->header.frame_id;
    has_cam_info_ = true;
    
    RCLCPP_INFO(this->get_logger(), "✅ Camera info received");
  }
  
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!has_cam_info_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "⚠️ Camera info not received yet!");
      return;
    }
    
    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    
    // Detect ArUco markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids);
    
    // Log detection status
    if (!ids.empty()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "✅ Detected %zu markers", ids.size());
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "❌ No ArUco markers detected");
    }
    
    // Always publish debug image (even if no markers detected)
    if (!ids.empty()) {
      // Estimate poses for all detected markers
      std::vector<cv::Vec3d> rvecs, tvecs;
      for (size_t i = 0; i < ids.size(); i++) {
        auto it = markers_.find(ids[i]);
        if (it == markers_.end()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "Detected marker %d not in config!", ids[i]);
          continue;
        }
        
        double marker_size = it->second.size;
        std::vector<cv::Point3f> obj_points = {
          cv::Point3f(-marker_size/2, marker_size/2, 0),
          cv::Point3f(marker_size/2, marker_size/2, 0),
          cv::Point3f(marker_size/2, -marker_size/2, 0),
          cv::Point3f(-marker_size/2, -marker_size/2, 0)
        };
        
        cv::Vec3d rvec, tvec;
        cv::solvePnP(obj_points, corners[i], K_, dist_coeffs_, rvec, tvec);
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
        
        // Calculate robot pose from marker detection
        calculateRobotPose(ids[i], rvec, tvec, msg->header.stamp);
      }
      
      // Draw detected markers
      cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
    }
    
    // Publish debug image
    auto debug_msg = cv_ptr->toImageMsg();
    pub_debug_image_->publish(*debug_msg);
  }
  
  void calculateRobotPose(int marker_id, const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                          const builtin_interfaces::msg::Time& stamp) {
    // Get marker position in map frame
    auto it = markers_.find(marker_id);
    if (it == markers_.end()) return;
    
    const MarkerInfo& marker = it->second;
    
    // Transform from camera to marker
    tf2::Transform T_cam_marker;
    tf2::Vector3 trans(tvec[0], tvec[1], tvec[2]);
    
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    tf2::Matrix3x3 rot(
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
    );
    
    T_cam_marker.setOrigin(trans);
    T_cam_marker.setBasis(rot);
    
    // Marker position/orientation in map frame
    tf2::Transform T_map_marker;
    T_map_marker.setOrigin(tf2::Vector3(marker.x, marker.y, marker.z));
    
    tf2::Quaternion q_marker;
    q_marker.setRPY(marker.roll, marker.pitch, marker.yaw);
    T_map_marker.setRotation(q_marker);
    
    // Calculate robot pose: T_map_robot = T_map_marker * T_marker_cam * T_cam_base
    // For simplicity, assuming camera is at robot base (adjust if needed)
    tf2::Transform T_marker_cam = T_cam_marker.inverse();
    tf2::Transform T_map_base = T_map_marker * T_marker_cam;
    
    // Publish robot pose in map frame
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "map";
    
    // Position
    pose_msg.pose.pose.position.x = T_map_base.getOrigin().x();
    pose_msg.pose.pose.position.y = T_map_base.getOrigin().y();
    pose_msg.pose.pose.position.z = T_map_base.getOrigin().z();
    
    // Orientation
    tf2::Quaternion q_base = T_map_base.getRotation();
    pose_msg.pose.pose.orientation.x = q_base.x();
    pose_msg.pose.pose.orientation.y = q_base.y();
    pose_msg.pose.pose.orientation.z = q_base.z();
    pose_msg.pose.pose.orientation.w = q_base.w();
    
    // Covariance - VERY LOW (ground truth!)
    double dist = tvec[0] * tvec[0] + tvec[1] * tvec[1] + tvec[2] * tvec[2];
    dist = std::sqrt(dist);
    
    double var_pos = pos_variance_ + dist * dist_scaling_;
    double var_ori = ori_variance_ + dist * dist_scaling_;
    
    // Covariance matrix (6x6): [x, y, z, rot_x, rot_y, rot_z]
    pose_msg.pose.covariance[0] = var_pos;   // x
    pose_msg.pose.covariance[7] = var_pos;   // y
    pose_msg.pose.covariance[14] = var_pos;  // z
    pose_msg.pose.covariance[21] = var_ori;  // rot_x
    pose_msg.pose.covariance[28] = var_ori;  // rot_y
    pose_msg.pose.covariance[35] = var_ori;  // rot_z
    
    pub_pose_cov_->publish(pose_msg);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "📍 Marker %d detected → Robot at (%.2f, %.2f) in map",
                         marker_id, pose_msg.pose.pose.position.x, 
                         pose_msg.pose.pose.position.y);
  }

  // Member variables
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  std::map<int, MarkerInfo> markers_;
  
  cv::Mat K_, dist_coeffs_;
  bool has_cam_info_ = false;
  std::string camera_frame_;
  
  double pos_variance_ = 0.0001;
  double ori_variance_ = 0.001;
  double dist_scaling_ = 0.00005;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_array_;
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
