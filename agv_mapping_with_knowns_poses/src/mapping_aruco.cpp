#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <set>

class ArucoSaverNode : public rclcpp::Node {
public:
  ArucoSaverNode()
  : Node("aruco_saver_node"),
    R_fix_((cv::Mat_<double>(3, 3) << 0, 0, 1, -1, 0, 0, 0, -1, 0)),
    dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)),
    has_cam_info_(false)
  {
    // Parameters
    image_topic_        = this->declare_parameter<std::string>("image_topic", "/zed2/left/image_raw");
    camera_info_topic_  = this->declare_parameter<std::string>("camera_info_topic", "/zed2/left/camera_info");
    marker_size_        = this->declare_parameter<double>("marker_size", 0.173);
    map_frame_id_       = this->declare_parameter<std::string>("map_frame", "map");
    cam_frame_id_       = this->declare_parameter<std::string>("camera_frame_id", "zed2_left_camera_frame");

    // TF Listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    auto sensor_qos = rclcpp::SensorDataQoS();
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos,
      std::bind(&ArucoSaverNode::cameraInfoCallback, this, std::placeholders::_1));

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, sensor_qos,
      std::bind(&ArucoSaverNode::imageCallback, this, std::placeholders::_1));

    // Setup save path
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("agv_mapping_with_knowns_poses");
    save_dir_ = pkg_share_dir + "/maps/aruco_markers";
    save_path_ = save_dir_ + "/aruco_markers.yaml";
    std::filesystem::create_directories(save_dir_);

    loadExistingMarkers();

    RCLCPP_INFO(this->get_logger(), "✅ ArUco Saver Node started. Will save new markers to %s", save_path_.c_str());
  }

private:
  void loadExistingMarkers() {
    std::ifstream fin(save_path_);
    if (!fin.good()) return;
    try {
        YAML::Node root = YAML::Load(fin);
        if (root["markers"]) {
            for (const auto& marker_node : root["markers"]) {
                saved_marker_ids_.insert(marker_node.first.as<int>());
            }
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse existing marker file: %s", e.what());
    }
    if (!saved_marker_ids_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Loaded %zu existing marker IDs. They will not be saved again.", saved_marker_ids_.size());
    }
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (has_cam_info_) return;
    K_ = (cv::Mat_<double>(3,3) << msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4], msg->k[5], msg->k[6], msg->k[7], msg->k[8]);
    if (!msg->d.empty()) {
      dist_coeffs_ = cv::Mat(msg->d).clone().reshape(1,1);
    } else {
      dist_coeffs_ = cv::Mat::zeros(1,5,CV_64F);
    }
    cam_frame_id_ = msg->header.frame_id.empty() ? cam_frame_id_ : msg->header.frame_id;
    has_cam_info_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera info received.");
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!has_cam_info_) return;

    cv::Mat frame_bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame_bgr, dictionary_, corners, ids);

    if (!ids.empty()) {
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, K_, dist_coeffs_, rvecs, tvecs);

      for (size_t i = 0; i < ids.size(); ++i) {
        if (saved_marker_ids_.count(ids[i])) continue;

        cv::Mat R_cv;
        cv::Rodrigues(rvecs[i], R_cv);
        cv::Mat R_ros = R_fix_ * R_cv;
        cv::Mat t_ros = R_fix_ * cv::Mat(tvecs[i]);
        
        tf2::Matrix3x3 tf_R(R_ros.at<double>(0,0), R_ros.at<double>(0,1), R_ros.at<double>(0,2),
                           R_ros.at<double>(1,0), R_ros.at<double>(1,1), R_ros.at<double>(1,2),
                           R_ros.at<double>(2,0), R_ros.at<double>(2,1), R_ros.at<double>(2,2));
        tf2::Quaternion q;
        tf_R.getRotation(q);

        geometry_msgs::msg::PoseStamped pose_in_cam_frame;
        pose_in_cam_frame.header.stamp = msg->header.stamp;
        pose_in_cam_frame.header.frame_id = cam_frame_id_;
        pose_in_cam_frame.pose.position.x = t_ros.at<double>(0);
        pose_in_cam_frame.pose.position.y = t_ros.at<double>(1);
        pose_in_cam_frame.pose.position.z = t_ros.at<double>(2);
        pose_in_cam_frame.pose.orientation = tf2::toMsg(q);

        transformAndSaveMarker(ids[i], pose_in_cam_frame);
      }
    }
  }

  void transformAndSaveMarker(int id, const geometry_msgs::msg::PoseStamped& pose_in_cam_frame) {
    geometry_msgs::msg::TransformStamped t_map_camera;
    try {
        t_map_camera = tf_buffer_->lookupTransform(map_frame_id_, cam_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform from '%s' to '%s': %s", cam_frame_id_.c_str(), map_frame_id_.c_str(), ex.what());
        return;
    }

    geometry_msgs::msg::PoseStamped pose_in_map_frame;
    tf2::doTransform(pose_in_cam_frame, pose_in_map_frame, t_map_camera);
    saveMarkerToYAML(id, pose_in_map_frame.pose);
  }

  void saveMarkerToYAML(int id, const geometry_msgs::msg::Pose &pose) {
    YAML::Node root;
    std::ifstream fin(save_path_);
    if (fin.good()) {
      root = YAML::Load(fin);
      fin.close();
    }

    YAML::Node marker;
    marker["position"]["x"] = pose.position.x;
    marker["position"]["y"] = pose.position.y;
    marker["position"]["z"] = pose.position.z;
    marker["orientation"]["x"] = pose.orientation.x;
    marker["orientation"]["y"] = pose.orientation.y;
    marker["orientation"]["z"] = pose.orientation.z;
    marker["orientation"]["w"] = pose.orientation.w;
    root["markers"][id] = marker;

    std::ofstream fout(save_path_);
    fout << root;
    fout.close();

    RCLCPP_INFO(this->get_logger(), "💾 Saved new marker %d pose in '%s' frame.", id, map_frame_id_.c_str());
    saved_marker_ids_.insert(id);
  }

  // Member variables
  std::string image_topic_, camera_info_topic_, map_frame_id_, cam_frame_id_;
  double marker_size_;
  const cv::Mat R_fix_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  bool has_cam_info_;
  cv::Mat K_, dist_coeffs_;
  std::set<int> saved_marker_ids_;
  std::string save_dir_, save_path_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoSaverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}