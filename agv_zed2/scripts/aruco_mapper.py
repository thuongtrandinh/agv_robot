// ============================================================================
// aruco_mapper.cpp - IPC Marker Mapper
//
// Inputs:
//   /aruco/detections            (Float32MultiArray from detector)
//   /aruco/pose_with_covariance  (map -> camera pose from localizer)
//
// Output:
//   aruco_map_positions.yaml     (marker poses in map)
//
// Formula:
//   T_map_marker = T_map_cam * T_cam_marker
//
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <map>
#include <set>
#include <fstream>

class ArucoMapperNode : public rclcpp::Node {
public:
    ArucoMapperNode()
    : Node("aruco_mapper_node")
    {
        map_frame_ = declare_parameter("map_frame", "map");
        map_file_  = declare_parameter("marker_map_file", "aruco_map_positions.yaml");

        std::string pkg = ament_index_cpp::get_package_share_directory("agv_zed2");
        save_path_ = pkg + "/map/" + map_file_;

        loadExistingMarkers();

        // --- subscribe từ detector ---
        sub_det_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aruco/detections", 10,
            std::bind(&ArucoMapperNode::detCallback, this, std::placeholders::_1));

        // --- subscribe pose camera từ localizer ---
        sub_cam_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/aruco/pose_with_covariance", 10,
            std::bind(&ArucoMapperNode::poseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "✅ Aruco Mapper started. Loaded %zu existing markers.",
                    saved_markers_.size());
    }

private:

    // ============================================================================
    // LOAD EXISTING YAML (resume mapping)
    // ============================================================================
    void loadExistingMarkers() {
        std::ifstream fin(save_path_);
        if (!fin.good()) return;

        YAML::Node yaml = YAML::Load(fin);
        fin.close();

        if (yaml["markers"]) {
            for (auto it : yaml["markers"]) {
                int id = it.first.as<int>();
                saved_markers_.insert(id);
            }
        }
    }

    // ============================================================================
    // CAMERA POSE CALLBACK (map → camera)
    // ============================================================================
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        tf2::fromMsg(msg->pose.pose, T_map_cam_);
        has_cam_pose_ = true;
    }

    // ============================================================================
    // DETECTOR CALLBACK (camera → marker)
    // ============================================================================
    void detCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (!has_cam_pose_) return;
        if (msg->data.size() % 8 != 0) return;

        int N = msg->data.size() / 8;

        for (int i = 0; i < N; i++) {
            int base = i * 8;
            int id = (int)msg->data[base];

            if (saved_markers_.count(id)) continue;

            // --- build T_cam_marker ---
            tf2::Transform T_cam_marker;
            T_cam_marker.setOrigin(tf2::Vector3(
                msg->data[base+1],
                msg->data[base+2],
                msg->data[base+3]
            ));

            tf2::Quaternion q(
                msg->data[base+4],
                msg->data[base+5],
                msg->data[base+6],
                msg->data[base+7]);
            T_cam_marker.setRotation(q);

            // --- compute map->marker ---
            tf2::Transform T_map_marker = T_map_cam_ * T_cam_marker;

            saveMarker(id, T_map_marker);

            saved_markers_.insert(id);
            break; // chỉ cần 1 marker mỗi frame
        }
    }

    // ============================================================================
    // SAVE MARKER TO YAML
    // ============================================================================
    void saveMarker(int id, const tf2::Transform &T)
    {
        YAML::Node root;

        std::ifstream fin(save_path_);
        if (fin.good()) root = YAML::Load(fin);
        fin.close();

        tf2::Quaternion q = T.getRotation();
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        YAML::Node mk;
        mk["position"]["x"] = T.getOrigin().x();
        mk["position"]["y"] = T.getOrigin().y();
        mk["position"]["z"] = T.getOrigin().z();
        mk["orientation"]["roll"]  = roll;
        mk["orientation"]["pitch"] = pitch;
        mk["orientation"]["yaw"]   = yaw;

        root["markers"][id] = mk;

        std::ofstream fout(save_path_);
        fout << root;
        fout.close();

        RCLCPP_INFO(get_logger(),
                    "💾 Saved marker %d at (%.2f, %.2f, %.2f), yaw=%.2f deg",
                    id,
                    T.getOrigin().x(),
                    T.getOrigin().y(),
                    T.getOrigin().z(),
                    yaw * 180.0 / M_PI);
    }

private:
    // --- parameters ---
    std::string map_frame_;
    std::string map_file_;
    std::string save_path_;

    // --- pose of camera ---
    tf2::Transform T_map_cam_;
    bool has_cam_pose_ = false;

    // --- marker storage ---
    std::set<int> saved_markers_;

    // --- subscribers ---
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_det_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_cam_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoMapperNode>());
    rclcpp::shutdown();
    return 0;
}
