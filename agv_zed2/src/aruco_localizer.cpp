// aruco_localizer.cpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <map>
#include <string>

struct MarkerInfo {
  int id;
  double x, y, z;
  double roll, pitch, yaw;
  double size;
};

class ArucoLocalizerNode : public rclcpp::Node
{
public:
  ArucoLocalizerNode()
  : Node("aruco_localizer_node")
  {
    // === Parameters ===
    map_frame_id_      = this->declare_parameter<std::string>("map_frame", "map");
    camera_frame_id_   = this->declare_parameter<std::string>("camera_frame_id", "zed2_left_camera_frame");
    marker_topic_      = this->declare_parameter<std::string>("marker_topic", "aruco/markers");
    marker_map_file_   = this->declare_parameter<std::string>("marker_map_file", "aruco_map_positions.yaml");

    pos_variance_      = this->declare_parameter<double>("position_variance", 0.0001);
    ori_variance_      = this->declare_parameter<double>("orientation_variance", 0.001);
    dist_scaling_      = this->declare_parameter<double>("distance_scaling", 0.00005);

    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("agv_mapping_with_knowns_poses");
    marker_map_path_ = pkg_share_dir + "/maps/aruco_markers/" + marker_map_file_;

    loadMarkerConfig(marker_map_path_);

    auto qos = rclcpp::SensorDataQoS();
    marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      marker_topic_, qos,
      std::bind(&ArucoLocalizerNode::markerCallback, this, std::placeholders::_1));

    pub_pose_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/aruco/pose_with_covariance", 10);

    RCLCPP_INFO(this->get_logger(),
                "✅ ArUco Localizer started. Map file: %s. Markers loaded: %zu",
                marker_map_path_.c_str(), markers_.size());
  }

private:
  void loadMarkerConfig(const std::string &config_file)
  {
    try {
      YAML::Node config = YAML::LoadFile(config_file);

      if (config["markers"]) {
        for (auto it : config["markers"]) {
          auto node = it.second;
          MarkerInfo mk;
          mk.id   = node["id"].as<int>();
          mk.x    = node["position"]["x"].as<double>();
          mk.y    = node["position"]["y"].as<double>();
          mk.z    = node["position"]["z"].as<double>();
          mk.roll = node["orientation"]["roll"].as<double>();
          mk.pitch= node["orientation"]["pitch"].as<double>();
          mk.yaw  = node["orientation"]["yaw"].as<double>();
          mk.size = node["size"].as<double>(0.173);
          markers_[mk.id] = mk;
        }
      }

    } catch (std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to load marker map: %s", e.what());
    }
  }

  void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    if (msg->markers.empty() || markers_.empty()) {
      return;
    }

    // Ta chỉ cần 1 marker đã biết để localize (có thể mở rộng fusion sau)
    for (const auto &marker : msg->markers) {
      int id = marker.id;
      if (markers_.count(id) == 0) {
        continue; // Marker chưa có trong map
      }

      // ===== map -> marker (known, từ YAML) =====
      const MarkerInfo &mk = markers_[id];

      tf2::Quaternion q_mk;
      q_mk.setRPY(mk.roll, mk.pitch, mk.yaw);

      tf2::Transform T_map_marker(q_mk, tf2::Vector3(mk.x, mk.y, mk.z));

      // ===== camera -> marker (measured, từ detector) =====
      tf2::Transform T_cam_marker;
      tf2::fromMsg(marker.pose, T_cam_marker);

      // ===== map -> camera =====
      tf2::Transform T_map_cam = T_map_marker * T_cam_marker.inverse();

      // Convert to PoseWithCovarianceStamped
      geometry_msgs::msg::PoseWithCovarianceStamped out;
      out.header.stamp = marker.header.stamp;
      out.header.frame_id = map_frame_id_;

      out.pose.pose.position.x = T_map_cam.getOrigin().x();
      out.pose.pose.position.y = T_map_cam.getOrigin().y();
      out.pose.pose.position.z = T_map_cam.getOrigin().z();
      auto q_cam = T_map_cam.getRotation();
      out.pose.pose.orientation = tf2::toMsg(q_cam);

      // Covariance scaling theo khoảng cách camera->marker
      double dist = T_cam_marker.getOrigin().length();
      double var_pos = pos_variance_ + dist * dist_scaling_;
      double var_ori = ori_variance_ + dist * dist_scaling_;

      for (int i = 0; i < 36; ++i) out.pose.covariance[i] = 0.0;
      out.pose.covariance[0]  = var_pos; // x
      out.pose.covariance[7]  = var_pos; // y
      out.pose.covariance[14] = var_pos; // z
      out.pose.covariance[21] = var_ori; // roll
      out.pose.covariance[28] = var_ori; // pitch
      out.pose.covariance[35] = var_ori; // yaw

      pub_pose_cov_->publish(out);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "📍 Marker %d → Camera at (%.2f, %.2f, %.2f), dist=%.2f",
        id,
        out.pose.pose.position.x,
        out.pose.pose.position.y,
        out.pose.pose.position.z,
        dist);

      // Dùng 1 marker là đủ, break (nếu muốn đa marker thì có thể average sau)
      break;
    }
  }

  // Members
  std::string map_frame_id_;
  std::string camera_frame_id_;
  std::string marker_topic_;
  std::string marker_map_file_;
  std::string marker_map_path_;

  double pos_variance_;
  double ori_variance_;
  double dist_scaling_;

  std::map<int, MarkerInfo> markers_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoLocalizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
