// ============================================================================
// aruco_localizer.cpp - IPC Localizer (works with aruco_detector.cpp)
// Subscribes: /aruco/detections (Float32MultiArray)
// Computes:   map -> camera pose
// Publishes:  /aruco/pose_with_covariance
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <map>

struct MarkerInfo {
    double x, y, z;
    double roll, pitch, yaw;
};

class ArucoLocalizerNode : public rclcpp::Node {
public:
    ArucoLocalizerNode()
    : Node("aruco_localizer_node")
    {
        map_frame_        = declare_parameter("map_frame", "map");
        marker_map_file_  = declare_parameter("marker_map_file", "aruco_map_positions.yaml");

        std::string pkg_share = ament_index_cpp::get_package_share_directory("agv_zed2");
        marker_map_path_ = pkg_share + "/map/" + marker_map_file_;

        loadMarkerMap();

        sub_det_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aruco/detections", 10,
            std::bind(&ArucoLocalizerNode::detCallback, this, std::placeholders::_1)
        );

        pub_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/aruco/pose_with_covariance", 10);

        RCLCPP_INFO(get_logger(), "✅ Aruco Localizer ready. Loaded %zu markers.", markers_.size());
    }

private:
    // ============================================================================
    // LOAD MARKER MAP (ZED2 calibration file)
    // ============================================================================
    void loadMarkerMap() {
        YAML::Node yaml = YAML::LoadFile(marker_map_path_);

        if (!yaml["markers"]) {
            RCLCPP_ERROR(get_logger(), "❌ No markers found in %s", marker_map_path_.c_str());
            return;
        }

        for (auto it : yaml["markers"]) {
            int id = it.first.as<int>();

            MarkerInfo mk;
            mk.x = it.second["position"]["x"].as<double>();
            mk.y = it.second["position"]["y"].as<double>();
            mk.z = it.second["position"]["z"].as<double>();
            mk.roll  = it.second["orientation"]["roll"].as<double>();
            mk.pitch = it.second["orientation"]["pitch"].as<double>();
            mk.yaw   = it.second["orientation"]["yaw"].as<double>();

            markers_[id] = mk;
        }

        RCLCPP_INFO(get_logger(), "📄 Loaded %zu markers from YAML.", markers_.size());
    }

    // ============================================================================
    // MAIN CALLBACK — compute map->camera using: T_map_marker * inverse(T_cam_marker)
    // ============================================================================
    void detCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 0) return;
        if (msg->data.size() % 8 != 0) return;

        int count = msg->data.size() / 8;

        for (int i = 0; i < count; i++)
        {
            int base = i * 8;

            int id = (int)msg->data[base];
            if (markers_.count(id) == 0)
                continue;

            // ============ T_cam_marker (from detector) ============
            tf2::Transform T_cam_marker;
            T_cam_marker.setOrigin(tf2::Vector3(
                msg->data[base + 1],
                msg->data[base + 2],
                msg->data[base + 3]
            ));

            tf2::Quaternion q_cam(
                msg->data[base + 4],
                msg->data[base + 5],
                msg->data[base + 6],
                msg->data[base + 7]
            );
            T_cam_marker.setRotation(q_cam);

            // ============ T_map_marker (from YAML) ============
            MarkerInfo &mk = markers_[id];

            tf2::Quaternion q_map;
            q_map.setRPY(mk.roll, mk.pitch, mk.yaw);

            tf2::Transform T_map_marker(q_map, tf2::Vector3(mk.x, mk.y, mk.z));

            // ============ MAIN FORMULA ============
            // T_map_cam = T_map_marker * inverse(T_cam_marker)
            tf2::Transform T_map_cam = T_map_marker * T_cam_marker.inverse();

            // ============ OUTPUT ROS MESSAGE ============
            geometry_msgs::msg::PoseWithCovarianceStamped out;
            out.header.stamp = now();
            out.header.frame_id = map_frame_;

            out.pose.pose.position.x = T_map_cam.getOrigin().x();
            out.pose.pose.position.y = T_map_cam.getOrigin().y();
            out.pose.pose.position.z = T_map_cam.getOrigin().z();
            out.pose.pose.orientation = tf2::toMsg(T_map_cam.getRotation());

            // === Simple covariance scaling ===
            double dist = T_cam_marker.getOrigin().length();
            double pvar = 0.0001 + dist * 0.0001;

            for (int k = 0; k < 36; k++) out.pose.covariance[k] = 0.0;
            out.pose.covariance[0]  = pvar;
            out.pose.covariance[7]  = pvar;
            out.pose.covariance[14] = pvar;
            out.pose.covariance[21] = 0.001;
            out.pose.covariance[28] = 0.001;
            out.pose.covariance[35] = 0.001;

            pub_pose_cov_->publish(out);

            // dùng 1 marker là đủ
            break;
        }
    }

private:
    // params
    std::string map_frame_;
    std::string marker_map_file_;
    std::string marker_map_path_;

    // marker map
    std::map<int, MarkerInfo> markers_;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_det_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
