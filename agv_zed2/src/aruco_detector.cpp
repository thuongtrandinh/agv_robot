// ============================================================================
// aruco_detector.cpp  (Native VGA 672×376, /dev/video4, 30 FPS)
// ============================================================================
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;

class ArucoDetectorNode : public rclcpp::Node {
public:
    ArucoDetectorNode()
        : Node("aruco_detector"),
          dict_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250))
    {
        calib_pkg_  = declare_parameter("calib_pkg", "agv_zed2");
        calib_file_ = declare_parameter("calib_file", "zed2_calibration_vga.yaml");
        marker_size_ = declare_parameter("marker_size", 0.18);
        device_ = declare_parameter("device", "/dev/video4");
        frame_id_ = declare_parameter("camera_frame", "zed2_left_camera_frame");

        loadCalibration();

        cap_.open(device_, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_FATAL(get_logger(), "❌ Cannot open device %s", device_.c_str());
            rclcpp::shutdown();
            return;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 672);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 376);
        cap_.set(cv::CAP_PROP_FPS, 30);

        RCLCPP_INFO(get_logger(), "📸 Camera %s opened (672×376 @ 30FPS)", device_.c_str());

        pub_detections_ = create_publisher<std_msgs::msg::Float32MultiArray>("/aruco/detections", 10);

        timer_ = create_wall_timer(
            33ms,
            std::bind(&ArucoDetectorNode::processFrame, this)
        );
    }

private:
    void loadCalibration() {
        std::string pkg = ament_index_cpp::get_package_share_directory(calib_pkg_);
        std::string path = pkg + "/config/" + calib_file_;

        YAML::Node yaml = YAML::LoadFile(path);
        auto cam = yaml["left_camera"];

        auto Kdata = cam["camera_matrix"]["data"].as<std::vector<double>>();
        auto Ddata = cam["distortion_coefficients"]["data"].as<std::vector<double>>();

        K_ = cv::Mat(3,3,CV_64F);
        for (int i=0;i<9;i++)
            K_.at<double>(i/3,i%3) = Kdata[i];

        D_ = cv::Mat(1,5,CV_64F);
        for (int i=0;i<5;i++)
            D_.at<double>(0,i) = Ddata[i];

        RCLCPP_INFO(get_logger(), "📂 Loaded VGA calibration: %s", path.c_str());
    }

    void processFrame() {
        cv::Mat img;
        cap_ >> img;
        if (img.empty()) return;

        // Chuyển sang grayscale
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        // Giảm kích thước ảnh (50%)
        cv::Mat small;
        cv::resize(gray, small, cv::Size(), 0.5, 0.5);

        // Xử lý song song, không block timer
        std::async(std::launch::async, [this, small]() {
            detectAruco(small);
        });
    }

    void detectAruco(const cv::Mat &img) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(img, dict_, corners, ids);
        if (ids.empty()) return;

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, K_, D_, rvecs, tvecs);

        std::map<int, size_t> id_to_best_idx;
        // Chọn detection gần camera nhất cho mỗi id
        for (size_t i = 0; i < ids.size(); i++) {
            double dist = cv::norm(tvecs[i]);
            if (id_to_best_idx.count(ids[i]) == 0 || dist < cv::norm(tvecs[id_to_best_idx[ids[i]]])) {
                id_to_best_idx[ids[i]] = i;
            }
        }

        std_msgs::msg::Float32MultiArray detections_msg;
        for (const auto& kv : id_to_best_idx) {
            size_t i = kv.second;
            cv::Mat R_cv;
            cv::Rodrigues(rvecs[i], R_cv);

            cv::Mat R_fix = (cv::Mat_<double>(3,3) <<
                0,0,1,
                -1,0,0,
                0,-1,0);

            cv::Mat R_ros = R_fix * R_cv;
            cv::Mat t_ros = R_fix * cv::Mat(tvecs[i]);

            tf2::Matrix3x3 m(
                R_ros.at<double>(0,0), R_ros.at<double>(0,1), R_ros.at<double>(0,2),
                R_ros.at<double>(1,0), R_ros.at<double>(1,1), R_ros.at<double>(1,2),
                R_ros.at<double>(2,0), R_ros.at<double>(2,1), R_ros.at<double>(2,2)
            );

            tf2::Quaternion q;
            m.getRotation(q);

            detections_msg.data.push_back(static_cast<float>(ids[i]));
            detections_msg.data.push_back(static_cast<float>(t_ros.at<double>(0)));
            detections_msg.data.push_back(static_cast<float>(t_ros.at<double>(1)));
            detections_msg.data.push_back(static_cast<float>(t_ros.at<double>(2)));
            detections_msg.data.push_back(static_cast<float>(q.x()));
            detections_msg.data.push_back(static_cast<float>(q.y()));
            detections_msg.data.push_back(static_cast<float>(q.z()));
            detections_msg.data.push_back(static_cast<float>(q.w()));
        }

        pub_detections_->publish(detections_msg);
    }

private:
    cv::Ptr<cv::aruco::Dictionary> dict_;
    cv::Mat K_, D_;
    cv::VideoCapture cap_;

    std::string calib_pkg_, calib_file_, device_, frame_id_;
    double marker_size_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_detections_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
