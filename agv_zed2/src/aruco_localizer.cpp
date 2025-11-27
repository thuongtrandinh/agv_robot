#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

// TF2 headers
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <map>
#include <string>
#include <memory>

struct MarkerInfo {
  int id;
  double x, y, z;
  double roll, pitch, yaw;
};

class ArucoLocalizerNode : public rclcpp::Node
{
public:
  ArucoLocalizerNode()
  : Node("aruco_localizer_node")
  {
    // === Parameters ===
    // Frame của bản đồ (Global)
    map_frame_id_      = this->declare_parameter<std::string>("map_frame", "map");
    // Frame của camera (Sensor)
    camera_frame_id_   = this->declare_parameter<std::string>("camera_frame_id", "zed2_left_camera_frame");
    // Frame của robot (Base) - QUAN TRỌNG ĐỂ EKF KHÔNG BỊ SAI
    base_frame_id_     = this->declare_parameter<std::string>("base_frame_id", "base_footprint");
    
    marker_map_file_   = this->declare_parameter<std::string>("marker_map_file", "aruco_map_positions.yaml");

    pos_variance_      = this->declare_parameter<double>("position_variance", 0.0001);
    ori_variance_      = this->declare_parameter<double>("orientation_variance", 0.001);
    dist_scaling_      = this->declare_parameter<double>("distance_scaling", 0.00005);
    
    // Tần số tối đa để publish (Tránh spam EKF)
    max_frequency_     = this->declare_parameter<double>("publish_frequency", 10.0); 

    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("agv_mapping_with_knowns_poses");
    marker_map_path_ = pkg_share_dir + "/maps/aruco_markers/" + marker_map_file_;

    loadMarkerConfig(marker_map_path_);

    // Setup TF Listener để lấy offset từ Base -> Camera
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto qos = rclcpp::SensorDataQoS(); // Best effort QoS cho Vision

    // Subscribe Detection
    detections_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/aruco/detections", qos,
      std::bind(&ArucoLocalizerNode::detectionsCallback, this, std::placeholders::_1));

    pub_pose_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/aruco/pose_with_covariance", 10);

    last_publish_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
                "✅ ArUco Localizer (Optimized). Rate Limit: %.1f Hz. Map: %s",
                max_frequency_, marker_map_file_.c_str());
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
          markers_[mk.id] = mk;
        }
      }
    } catch (std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to load marker map: %s", e.what());
    }
  }

  void detectionsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const auto &d = msg->data;
    if (d.empty()) return;

    // === OPTIMIZATION 1: RATE LIMITING ===
    // Chỉ xử lý nếu đủ thời gian trôi qua (Giảm tải CPU)
    rclcpp::Time now = this->now();
    double dt = (now - last_publish_time_).seconds();
    if (dt < (1.0 / max_frequency_)) {
      return; 
    }

    // === OPTIMIZATION 2: Xử lý trực tiếp, bỏ qua bước tạo MarkerArray trung gian ===
    // Duyệt qua các marker detect được
    for (size_t i = 0; i + 7 < d.size(); i += 8) {
      int id = static_cast<int>(d[i]);
      
      // Chỉ xử lý nếu marker này có trong Database
      if (markers_.count(id) == 0) continue;

      processMarker(id, d[i+1], d[i+2], d[i+3], d[i+4], d[i+5], d[i+6], d[i+7], now);
      
      // Chỉ cần 1 marker tốt nhất là đủ để Localization.
      // Break ngay để tiết kiệm CPU, không cần tính hết cho các marker khác.
      last_publish_time_ = now;
      break; 
    }
  }

  void processMarker(int id, double tx, double ty, double tz, 
                     double qx, double qy, double qz, double qw, rclcpp::Time stamp)
  {
    // 1. Lấy tọa độ Marker trên Map (Known)
    const MarkerInfo &mk = markers_[id];
    tf2::Quaternion q_mk_map;
    q_mk_map.setRPY(mk.roll, mk.pitch, mk.yaw);
    tf2::Transform T_map_marker(q_mk_map, tf2::Vector3(mk.x, mk.y, mk.z));

    // 2. Lấy tọa độ Marker so với Camera (Measured)
    tf2::Quaternion q_cam_marker(qx, qy, qz, qw);
    tf2::Vector3 v_cam_marker(tx, ty, tz);
    tf2::Transform T_cam_marker(q_cam_marker, v_cam_marker);

    // 3. Tính tọa độ Camera trên Map: T_map_cam = T_map_marker * T_marker_cam
    // Lưu ý: T_marker_cam = T_cam_marker^-1
    tf2::Transform T_map_cam = T_map_marker * T_cam_marker.inverse();

    // === OPTIMIZATION 3: Transform về Base Link (Robot Center) ===
    // EKF cần biết robot đang ở đâu, chứ không phải camera đang ở đâu.
    tf2::Transform T_base_cam;
    try {
      // Tìm transform tĩnh từ Base -> Camera (Ví dụ: Camera gắn trước robot 20cm)
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg = tf_buffer_->lookupTransform(base_frame_id_, camera_frame_id_, tf2::TimePointZero);
      tf2::fromMsg(tf_msg.transform, T_base_cam);
    } catch (tf2::TransformException &ex) {
      // Nếu chưa có TF (lúc khởi động), dùng Identity (coi như Camera trùng tâm Robot)
      T_base_cam.setIdentity();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        "⚠️ No TF from %s to %s. Assuming Identity.", base_frame_id_.c_str(), camera_frame_id_.c_str());
    }

    // T_map_base = T_map_cam * T_base_cam^-1
    // (Vị trí Robot = Vị trí Camera lùi lại một đoạn offset)
    tf2::Transform T_map_base = T_map_cam * T_base_cam.inverse();

    // 4. Tạo message PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = map_frame_id_;

    out.pose.pose.position.x = T_map_base.getOrigin().x();
    out.pose.pose.position.y = T_map_base.getOrigin().y();
    out.pose.pose.position.z = 0.0; // Robot AGV thường chạy trên mặt phẳng, ép Z=0 cho an toàn
    
    // Chỉ lấy Yaw cho Robot, bỏ qua Roll/Pitch nếu xe chạy trong nhà
    double roll, pitch, yaw;
    T_map_base.getRotation().getRPY(roll, pitch, yaw);
    tf2::Quaternion q_final;
    q_final.setRPY(0, 0, yaw); // Ép về 2D mode
    out.pose.pose.orientation = tf2::toMsg(q_final);

    // 5. Tính Covariance (Độ tin cậy)
    // Khoảng cách càng xa, độ tin cậy càng thấp (Covariance càng cao)
    double dist = v_cam_marker.length();
    double var_pos = pos_variance_ + (dist * dist * dist_scaling_);
    double var_ori = ori_variance_ + (dist * dist * dist_scaling_);

    // Fill ma trận hiệp phương sai 6x6
    for (int i = 0; i < 36; ++i) out.pose.covariance[i] = 0.0;
    out.pose.covariance[0]  = var_pos; // x
    out.pose.covariance[7]  = var_pos; // y
    out.pose.covariance[14] = var_pos; // z
    out.pose.covariance[21] = var_ori; // roll
    out.pose.covariance[28] = var_ori; // pitch
    out.pose.covariance[35] = var_ori; // yaw

    pub_pose_cov_->publish(out);
  }

  // Members
  std::string map_frame_id_;
  std::string camera_frame_id_;
  std::string base_frame_id_;
  std::string marker_map_file_;
  std::string marker_map_path_;

  double pos_variance_;
  double ori_variance_;
  double dist_scaling_;
  double max_frequency_;
  rclcpp::Time last_publish_time_;

  std::map<int, MarkerInfo> markers_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoLocalizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}