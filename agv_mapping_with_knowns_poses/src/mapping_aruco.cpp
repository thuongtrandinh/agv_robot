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

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <set>
#include <cmath>

class ArucoSaverNode : public rclcpp::Node {
public:
  ArucoSaverNode()
  : Node("aruco_saver_node"),
    R_fix_((cv::Mat_<double>(3, 3) << 0, 0, 1,
                                      -1, 0, 0,
                                       0, -1, 0)),
    dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)),
    has_cam_info_(false)
  {
    // Parameters
    image_topic_        = this->declare_parameter<std::string>("image_topic", "/zed2/left/image_raw");
    camera_info_topic_  = this->declare_parameter<std::string>("camera_info_topic", "/zed2/left/camera_info");
    marker_size_        = this->declare_parameter<double>("marker_size", 0.173);
    map_frame_id_       = this->declare_parameter<std::string>("map_frame", "map");
    cam_frame_id_       = this->declare_parameter<std::string>("camera_frame_id", "zed2_left_camera_frame");

    max_marker_distance_ = this->declare_parameter<double>("max_marker_distance", 3.0); // reject > 3m
    max_jump_distance_   = this->declare_parameter<double>("max_jump_distance", 0.30); // reject > 30cm
    max_yaw_jump_        = this->declare_parameter<double>("max_yaw_jump", 0.5); // rad (~30°)

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

    RCLCPP_INFO(this->get_logger(), "ArucoSaverNode started. Saving to %s", save_path_.c_str());
  }

private:

  // ============================
  // LOAD EXISTING MARKERS
  // ============================
  void loadExistingMarkers() {
    std::ifstream fin(save_path_);
    if (!fin.good()) return;

    try {
      YAML::Node root = YAML::Load(fin);
      if (root["markers"]) {
        for (auto it : root["markers"]) {
          saved_marker_ids_.insert(it.first.as<int>());
        }
      }
    } catch (...) {}

    if (!saved_marker_ids_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Loaded %zu existing markers.", saved_marker_ids_.size());
    }
  }

  // ============================
  // CAMERA INFO CALLBACK
  // ============================
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (has_cam_info_) return;

    K_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();

    if (!msg->d.empty())
      dist_coeffs_ = cv::Mat(msg->d).clone().reshape(1,1);
    else
      dist_coeffs_ = cv::Mat::zeros(1,5,CV_64F);

    cam_frame_id_ = msg->header.frame_id;
    has_cam_info_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera info received.");
  }

  // ============================
  // IMAGE CALLBACK
  // ============================
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!has_cam_info_) return;

    cv::Mat frame_bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(frame_bgr, dictionary_, corners, ids);

    if (ids.empty()) return;

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, K_, dist_coeffs_, rvecs, tvecs);

    for (size_t i = 0; i < ids.size(); i++) {
      int marker_id = ids[i];

      if (saved_marker_ids_.count(marker_id)) continue;

      double dist = cv::norm(tvecs[i]);
      if (dist > max_marker_distance_) {
        RCLCPP_WARN(this->get_logger(), "Reject marker %d (distance too large: %.2f m)", marker_id, dist);
        continue;
      }

      cv::Mat R_cv;
      cv::Rodrigues(rvecs[i], R_cv);

      cv::Mat R_ros = R_fix_ * R_cv;
      cv::Mat t_ros = R_fix_ * cv::Mat(tvecs[i]);

      tf2::Matrix3x3 tf_R(
        R_ros.at<double>(0,0), R_ros.at<double>(0,1), R_ros.at<double>(0,2),
        R_ros.at<double>(1,0), R_ros.at<double>(1,1), R_ros.at<double>(1,2),
        R_ros.at<double>(2,0), R_ros.at<double>(2,1), R_ros.at<double>(2,2)
      );
      tf2::Quaternion q;  
      tf_R.getRotation(q);

      geometry_msgs::msg::PoseStamped pose_cam;
      pose_cam.header = msg->header;
      pose_cam.header.frame_id = cam_frame_id_;
      pose_cam.pose.position.x = t_ros.at<double>(0);
      pose_cam.pose.position.y = t_ros.at<double>(1);
      pose_cam.pose.position.z = t_ros.at<double>(2);
      pose_cam.pose.orientation = tf2::toMsg(q);

      transformAndSaveMarker(marker_id, pose_cam);
    }
  }

  // ============================
  // TRANSFORM FROM CAMERA → MAP
  // ============================
  void transformAndSaveMarker(int id, const geometry_msgs::msg::PoseStamped& pose_in_cam_frame) {
    geometry_msgs::msg::TransformStamped T_map_cam;

    try {
      T_map_cam = tf_buffer_->lookupTransform(
        map_frame_id_, 
        cam_frame_id_, 
        pose_in_cam_frame.header.stamp,
        rclcpp::Duration::from_seconds(0.1)
      );
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped pose_map;
    tf2::doTransform(pose_in_cam_frame, pose_map, T_map_cam);

    // Reject sudden jumps
    if (last_marker_poses_.count(id)) {
        auto &prev = last_marker_poses_[id];
        double dx = pose_map.pose.position.x - prev.pose.position.x;
        double dy = pose_map.pose.position.y - prev.pose.position.y;
        double jump = std::sqrt(dx*dx + dy*dy);

        if (jump > max_jump_distance_) {
          RCLCPP_WARN(this->get_logger(), "Reject marker %d (jump too big %.2f m)", id, jump);
          return;
        }
    }

    last_marker_poses_[id] = pose_map;
    saveMarkerToYAML(id, pose_map.pose);
  }

  // ============================
  // SAVE TO YAML (RPY VERSION)
  // ============================
  void saveMarkerToYAML(int id, const geometry_msgs::msg::Pose &pose) {
    YAML::Node root;

    std::ifstream fin(save_path_);
    if (fin.good()) {
      root = YAML::Load(fin);
      fin.close();
    }

    // Convert quaternion → rpy
    tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    YAML::Node marker;
    marker["id"] = id;
    marker["position"]["x"] = pose.position.x;
    marker["position"]["y"] = pose.position.y;
    marker["position"]["z"] = pose.position.z;

    marker["orientation"]["roll"]  = roll;
    marker["orientation"]["pitch"] = pitch;
    marker["orientation"]["yaw"]   = yaw;

    marker["size"] = marker_size_;

    root["markers"][id] = marker;

    std::ofstream fout(save_path_);
    fout << root;
    fout.close();

    RCLCPP_INFO(this->get_logger(), "💾 Marker %d saved: (%.2f, %.2f) yaw=%.2f", id,
                pose.position.x, pose.position.y, yaw);

    saved_marker_ids_.insert(id);
  }

  // ============================
  // MEMBER VARIABLES
  // ============================
  std::string image_topic_, camera_info_topic_, map_frame_id_, cam_frame_id_;
  double marker_size_;
  double max_marker_distance_;
  double max_jump_distance_;
  double max_yaw_jump_;

  const cv::Mat R_fix_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  bool has_cam_info_;
  cv::Mat K_, dist_coeffs_;

  std::set<int> saved_marker_ids_;
  std::map<int, geometry_msgs::msg::PoseStamped> last_marker_poses_;

  std::string save_dir_, save_path_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoSaverNode>());
  rclcpp::shutdown();
  return 0;
}
