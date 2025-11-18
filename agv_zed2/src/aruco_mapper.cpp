// aruco_mapper.cpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <fstream>
#include <map>
#include <set>

class ArucoMapperNode : public rclcpp::Node
{
public:
  ArucoMapperNode()
  : Node("aruco_mapper_node")
  {
    // ===== Parameters =====
    map_frame_id_    = this->declare_parameter<std::string>("map_frame", "map");
    camera_frame_id_ = this->declare_parameter<std::string>("camera_frame_id", "zed2_left_camera_frame");
    marker_topic_    = this->declare_parameter<std::string>("marker_topic", "aruco/markers");
    marker_size_     = this->declare_parameter<double>("marker_size", 0.18);

    max_marker_distance_ = this->declare_parameter<double>("max_marker_distance", 3.0);
    max_jump_distance_   = this->declare_parameter<double>("max_jump_distance", 0.30);

    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("agv_mapping_with_knowns_poses");
    save_dir_  = pkg_share_dir + "/maps/aruco_markers";
    save_path_ = save_dir_ + "/aruco_map_positions.yaml";
    std::filesystem::create_directories(save_dir_);

    // TF Buffer & Listener
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe MarkerArray
    auto qos = rclcpp::SensorDataQoS();
    marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      marker_topic_, qos,
      std::bind(&ArucoMapperNode::markerCallback, this, std::placeholders::_1));

    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/aruco/markers", 10);

    loadExistingMarkers();

    RCLCPP_INFO(this->get_logger(), "✅ ArUco Mapper started. Saving to: %s",
                save_path_.c_str());
  }

private:
  // ==== Load existing markers from YAML (resume mapping) ====
  void loadExistingMarkers()
  {
    std::ifstream fin(save_path_);
    if (!fin.good()) {
      return;
    }

    try {
      YAML::Node root = YAML::Load(fin);
      fin.close();
      if (root["markers"]) {
        for (auto it : root["markers"]) {
          int id = it.second["id"].as<int>();
          saved_marker_ids_.insert(id);
        }
      }

      if (!saved_marker_ids_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Loaded %zu existing markers.", saved_marker_ids_.size());
      }
    } catch (...) {
      RCLCPP_WARN(this->get_logger(), "Failed to parse existing YAML marker file.");
    }
  }

  // ==== Marker callback ====
  void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    if (msg->markers.empty()) {
      return;
    }

    // Lấy frame camera từ header
    std::string cam_frame = msg->markers[0].header.frame_id.empty() ?
                            camera_frame_id_ : msg->markers[0].header.frame_id;

    // Lookup TF: map -> camera
    geometry_msgs::msg::TransformStamped T_map_cam;
    try {
      T_map_cam = tf_buffer_->lookupTransform(
        map_frame_id_, cam_frame,
        msg->markers[0].header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed (map->%s): %s", cam_frame.c_str(), ex.what());
      return;
    }

    // Convert T_map_cam to tf2::Transform
    tf2::Transform tf_map_cam;
    tf2::fromMsg(T_map_cam.transform, tf_map_cam);

    for (const auto & marker : msg->markers) {
      int id = marker.id;
      if (saved_marker_ids_.count(id)) {
        continue; // đã lưu marker này rồi
      }

      // Pose marker trong camera frame
      tf2::Transform tf_cam_marker;
      tf2::fromMsg(marker.pose, tf_cam_marker);

      // Reject distance quá xa
      double dist = tf_cam_marker.getOrigin().length();
      if (dist > max_marker_distance_) {
        RCLCPP_WARN(this->get_logger(),
                    "Reject marker %d (distance %.2f m > %.2f m)",
                    id, dist, max_marker_distance_);
        continue;
      }

      // T_map_marker = T_map_cam * T_cam_marker
      tf2::Transform tf_map_marker = tf_map_cam * tf_cam_marker;

      // Chuyển sang Pose trong map
      geometry_msgs::msg::Pose pose_map;
      pose_map.position.x = tf_map_marker.getOrigin().x();
      pose_map.position.y = tf_map_marker.getOrigin().y();
      pose_map.position.z = tf_map_marker.getOrigin().z();
      auto q = tf_map_marker.getRotation();
      pose_map.orientation = tf2::toMsg(q);

      // Reject jump nếu đã có marker cũ trong session (không phải file)
      if (last_marker_poses_.count(id)) {
        auto &prev = last_marker_poses_[id];
        double dx = pose_map.position.x - prev.position.x;
        double dy = pose_map.position.y - prev.position.y;
        double jump = std::sqrt(dx*dx + dy*dy);
        if (jump > max_jump_distance_) {
          RCLCPP_WARN(this->get_logger(),
                      "Reject marker %d (jump %.2f m > %.2f m)",
                      id, jump, max_jump_distance_);
          continue;
        }
      }

      last_marker_poses_[id] = pose_map;
      saveMarkerToYAML(id, pose_map);
    }

    publishMarkers(msg);
  }

  void publishMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray markers_msg;
    rclcpp::Time now = this->now();

    for (size_t i = 0; i < msg->markers.size(); i++) {
      const auto &marker = msg->markers[i];

      visualization_msgs::msg::Marker m;
      m.header.stamp = now;
      m.header.frame_id = marker.header.frame_id.empty() ? camera_frame_id_ : marker.header.frame_id;
      m.ns = "aruco";
      m.id = marker.id;

      m.pose = marker.pose;

      // marker scale (optional, used only for visualization)
      m.scale.x = marker_size_;
      m.scale.y = marker_size_;
      m.scale.z = 0.01;

      markers_msg.markers.push_back(m);
    }

    pub_markers_->publish(markers_msg);
  }

  // ==== Save marker to YAML (RPY version) ====
  void saveMarkerToYAML(int id, const geometry_msgs::msg::Pose &pose)
  {
    YAML::Node root;
    std::ifstream fin(save_path_);
    if (fin.good()) {
      root = YAML::Load(fin);
      fin.close();
    }

    // quaternion -> RPY
    tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
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

    saved_marker_ids_.insert(id);

    RCLCPP_INFO(this->get_logger(),
                "💾 Saved marker %d at (%.2f, %.2f, %.2f), yaw=%.2f deg",
                id,
                pose.position.x, pose.position.y, pose.position.z,
                yaw * 180.0 / M_PI);
  }

  // ==== Members ====
  std::string map_frame_id_;
  std::string camera_frame_id_;
  std::string marker_topic_;

  double marker_size_;
  double max_marker_distance_;
  double max_jump_distance_;

  std::string save_dir_;
  std::string save_path_;

  std::set<int> saved_marker_ids_;
  std::map<int, geometry_msgs::msg::Pose> last_marker_poses_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoMapperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
