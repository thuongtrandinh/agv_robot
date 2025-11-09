#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <limits>
#include <map>

class StereoCpuVGA60Node : public rclcpp::Node {
public:
  StereoCpuVGA60Node() : Node("mapping") {
    // --- Params (default: 1344x376 @ 60Hz SBS -> mỗi mắt 672x376)
    device_id_    = this->declare_parameter<int>("device_id", 0);
    frame_width_  = this->declare_parameter<int>("frame_width", 1344);
    frame_height_ = this->declare_parameter<int>("frame_height", 376);
    marker_size_  = this->declare_parameter<double>("marker_size", 0.173); // m
    calib_file_   = this->declare_parameter<std::string>("calib_file", "zed2_stereo_vga.yaml");
    save_yaml_    = this->declare_parameter<bool>("save_yaml", false);

    // --- Publishers
    auto sensor_qos = rclcpp::SensorDataQoS();
    pub_left_   = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_rect", sensor_qos);
    pub_right_  = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_rect", sensor_qos);
    pub_depth_  = this->create_publisher<sensor_msgs::msg::Image>("/stereo/depth/image", sensor_qos);
    pub_debug_  = this->create_publisher<sensor_msgs::msg::Image>("/aruco/image", 5);
    pub_pose_   = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/pose", 20);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/aruco/marker_vis", 20);

    // --- Load calibration
    if (!loadCalibration(calib_file_)) {
      RCLCPP_FATAL(get_logger(), "Cannot load calibration config '%s'", calib_file_.c_str());
      throw std::runtime_error("Missing calibration");
    }
    computeRectify();

    // --- Open UVC (SBS)
    cap_.open(device_id_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_FATAL(get_logger(), "Cannot open /dev/video%d", device_id_);
      throw std::runtime_error("Camera open failed");
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    cap_.set(cv::CAP_PROP_FPS, 60);

    // --- Stereo matcher (tối ưu CPU cho VGA)
    matcher_ = cv::StereoSGBM::create(
      /*minDisparity*/ 0,
      /*numDisparities*/ 96,          // bội số 16 (64~128 tuỳ CPU)
      /*blockSize*/ 5,
      /*P1*/ 8 * 3 * 5 * 5,
      /*P2*/ 32 * 3 * 5 * 5,
      /*disp12MaxDiff*/ 1,
      /*preFilterCap*/ 31,
      /*uniquenessRatio*/ 10,
      /*speckleWindowSize*/ 50,
      /*speckleRange*/ 2,
      /*mode*/ cv::StereoSGBM::MODE_SGBM_3WAY
    );

    dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // --- YAML save path (nếu bật)
    if (save_yaml_) {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("agv_zed2");
      save_path_ = pkg_share + "/map/aruco_markers.yaml";
      std::filesystem::create_directories(pkg_share + "/map");
      RCLCPP_INFO(get_logger(), "Saving poses to: %s", save_path_.c_str());
    }

    // --- Timer 60 Hz
    timer_ = this->create_wall_timer(std::chrono::milliseconds(16),
              std::bind(&StereoCpuVGA60Node::loop, this));

    RCLCPP_INFO(get_logger(), "✅ Started (device=%d, %dx%d @ 60Hz SBS)", device_id_, frame_width_, frame_height_);
  }

  ~StereoCpuVGA60Node() {
    if (save_yaml_ && !detected_markers_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Node shutting down. Saving %zu markers to YAML...", detected_markers_.size());
      saveMapToYAML();
      RCLCPP_INFO(this->get_logger(), "💾 Map saved successfully to %s", save_path_.c_str());
    }
  }

private:
  // Params
  int device_id_, frame_width_, frame_height_;
  double marker_size_;
  std::string calib_file_;
  bool save_yaml_;
  std::string save_path_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_, pub_right_, pub_depth_, pub_debug_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::TimerBase::SharedPtr timer_;

  // OpenCV
  cv::VideoCapture cap_;
  cv::Ptr<cv::StereoSGBM> matcher_;
  cv::Ptr<cv::aruco::Dictionary> dict_;

  // Calibration/rectify
  cv::Mat K1_, D1_, K2_, D2_, R_, T_;
  cv::Mat R1_, R2_, P1_, P2_, Q_;
  cv::Mat map1x_, map1y_, map2x_, map2y_;

  // Marker map
  std::map<int, geometry_msgs::msg::Pose> detected_markers_;

  bool loadCalibration(const std::string &fname) {
    std::string pkg_share = ament_index_cpp::get_package_share_directory("agv_zed2");
    std::string path = pkg_share + "/config/" + fname;
    if (!std::filesystem::exists(path)) return false;

    YAML::Node c = YAML::LoadFile(path);
    auto mL = c["camera_matrix_left"].as<std::vector<std::vector<double>>>();
    auto dL = c["dist_coeffs_left"].as<std::vector<double>>();
    auto mR = c["camera_matrix_right"].as<std::vector<std::vector<double>>>();
    auto dR = c["dist_coeffs_right"].as<std::vector<double>>();
    auto RR = c["R"].as<std::vector<std::vector<double>>>();
    auto TT = c["T"].as<std::vector<double>>();

    K1_ = cv::Mat(3,3,CV_64F); K2_ = cv::Mat(3,3,CV_64F);
    for (int i=0;i<3;i++) for (int j=0;j<3;j++){ K1_.at<double>(i,j)=mL[i][j]; K2_.at<double>(i,j)=mR[i][j]; }
    D1_ = cv::Mat(dL).clone().reshape(1,1);
    D2_ = cv::Mat(dR).clone().reshape(1,1);
    R_  = cv::Mat(3,3,CV_64F);
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) R_.at<double>(i,j)=RR[i][j];
    T_  = (cv::Mat_<double>(3,1) << TT[0], TT[1], TT[2]);

    RCLCPP_INFO(get_logger(), "Loaded calib: %s", path.c_str());
    return true;
  }

  void computeRectify() {
    cv::Size sz(frame_width_/2, frame_height_); // mỗi mắt
    cv::stereoRectify(K1_, D1_, K2_, D2_, sz, R_, T_, R1_, R2_, P1_, P2_, Q_,
                      cv::CALIB_ZERO_DISPARITY, 0, sz);
    cv::initUndistortRectifyMap(K1_, D1_, R1_, P1_, sz, CV_32FC1, map1x_, map1y_);
    cv::initUndistortRectifyMap(K2_, D2_, R2_, P2_, sz, CV_32FC1, map2x_, map2y_);
  }

  void loop() {
    cv::Mat sbs;
    if (!cap_.read(sbs)) return;

    // Resize về đúng cấu hình (phòng khi driver scale)
    if (sbs.cols != frame_width_ || sbs.rows != frame_height_)
      cv::resize(sbs, sbs, cv::Size(frame_width_, frame_height_));

    // Tách trái/phải
    int w = frame_width_/2, h = frame_height_;
    cv::Mat left_raw  = sbs(cv::Rect(0, 0, w, h)).clone();
    cv::Mat right_raw = sbs(cv::Rect(w, 0, w, h)).clone();

    // Rectify
    cv::Mat left, right;
    cv::remap(left_raw,  left,  map1x_, map1y_, cv::INTER_LINEAR);
    cv::remap(right_raw, right, map2x_, map2y_, cv::INTER_LINEAR);

    // Disparity (CPU)
    cv::Mat gL, gR;
    cv::cvtColor(left, gL, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right, gR, cv::COLOR_BGR2GRAY);

    cv::Mat disp16;
    matcher_->compute(gL, gR, disp16);

    cv::Mat disp;
    disp16.convertTo(disp, CV_32F, 1.0/16.0);

    // Reproject -> 3D
    cv::Mat points3D; // CV_32FC3 (m)
    cv::reprojectImageTo3D(disp, points3D, Q_, false);

    // ArUco detect (trên ảnh trái đã rectify)
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(left, dict_, corners, ids);

    // K rectified từ P1
    cv::Mat Krect = (cv::Mat_<double>(3,3) <<
      P1_.at<double>(0,0), 0, P1_.at<double>(0,2),
      0, P1_.at<double>(1,1), P1_.at<double>(1,2),
      0, 0, 1
    );
    cv::Mat Dzero = cv::Mat::zeros(1,5,CV_64F);

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(left, corners, ids);

      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, Krect, Dzero, rvecs, tvecs);

      for (size_t i=0;i<ids.size(); ++i) {
        // Lấy 3D tại tâm marker
        int cx = int((corners[i][0].x + corners[i][2].x) * 0.5);
        int cy = int((corners[i][0].y + corners[i][2].y) * 0.5);
        cx = std::clamp(cx, 0, w-1);
        cy = std::clamp(cy, 0, h-1);
        cv::Vec3f XYZ = points3D.at<cv::Vec3f>(cy, cx);

        // Orientation từ rvec (đổi trục sang ROS: X fwd, Y left, Z up)
        cv::Mat Rcv; cv::Rodrigues(rvecs[i], Rcv);
        cv::Mat R_fix = (cv::Mat_<double>(3,3) <<
             1,  0,  0,
             0, -1,  0,
             0,  0, -1
        );
        cv::Mat Rros = R_fix * Rcv;

        // R -> quaternion
        double tr = Rros.at<double>(0,0)+Rros.at<double>(1,1)+Rros.at<double>(2,2);
        double qw,qx,qy,qz;
        if (tr > 0) {
          double S = std::sqrt(tr+1.0)*2.0;
          qw = 0.25*S;
          qx = (Rros.at<double>(2,1)-Rros.at<double>(1,2))/S;
          qy = (Rros.at<double>(0,2)-Rros.at<double>(2,0))/S;
          qz = (Rros.at<double>(1,0)-Rros.at<double>(0,1))/S;
        } else if ((Rros.at<double>(0,0) > Rros.at<double>(1,1)) && (Rros.at<double>(0,0) > Rros.at<double>(2,2))) {
          double S = std::sqrt(1.0 + Rros.at<double>(0,0) - Rros.at<double>(1,1) - Rros.at<double>(2,2))*2.0;
          qw = (Rros.at<double>(2,1)-Rros.at<double>(1,2))/S;
          qx = 0.25*S;
          qy = (Rros.at<double>(0,1)+Rros.at<double>(1,0))/S;
          qz = (Rros.at<double>(0,2)+Rros.at<double>(2,0))/S;
        } else if (Rros.at<double>(1,1) > Rros.at<double>(2,2)) {
          double S = std::sqrt(1.0 + Rros.at<double>(1,1) - Rros.at<double>(0,0) - Rros.at<double>(2,2))*2.0;
          qw = (Rros.at<double>(0,2)-Rros.at<double>(2,0))/S;
          qx = (Rros.at<double>(0,1)+Rros.at<double>(1,0))/S;
          qy = 0.25*S;
          qz = (Rros.at<double>(1,2)+Rros.at<double>(2,1))/S;
        } else {
          double S = std::sqrt(1.0 + Rros.at<double>(2,2) - Rros.at<double>(0,0) - Rros.at<double>(1,1))*2.0;
          qw = (Rros.at<double>(1,0)-Rros.at<double>(0,1))/S;
          qx = (Rros.at<double>(0,2)+Rros.at<double>(2,0))/S;
          qy = (Rros.at<double>(1,2)+Rros.at<double>(2,1))/S;
          qz = 0.25*S;
        }

        // Publish pose (dùng XYZ từ stereo, orientation từ PnP)
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = this->now();
        ps.header.frame_id = "stereo_left_camera_frame";
        ps.pose.position.x = XYZ[0];
        ps.pose.position.y = XYZ[1];
        ps.pose.position.z = XYZ[2];
        ps.pose.orientation.x = qx;
        ps.pose.orientation.y = qy;
        ps.pose.orientation.z = qz;
        ps.pose.orientation.w = qw;
        pub_pose_->publish(ps);

        // RViz marker
        visualization_msgs::msg::Marker mk;
        mk.header = ps.header;
        mk.ns = "aruco";
        mk.id = ids[i];
        mk.type = visualization_msgs::msg::Marker::CUBE;
        mk.action = visualization_msgs::msg::Marker::ADD;
        mk.pose = ps.pose;
        mk.scale.x = marker_size_;
        mk.scale.y = marker_size_;
        mk.scale.z = 0.01;
        mk.color.r = 0.0f; mk.color.g = 1.0f; mk.color.b = 0.0f; mk.color.a = 0.85f;
        pub_marker_->publish(mk);

        // Vẽ debug
        cv::aruco::drawAxis(left, Krect, Dzero, rvecs[i], tvecs[i], marker_size_*0.4);
        cv::putText(left,
          "ID="+std::to_string(ids[i])+" d="+cv::format("%.2fm", cv::norm(XYZ)),
          cv::Point(cx-40, cy-10), cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,255}, 2);

        // Cập nhật pose của marker vào map trong bộ nhớ
        if (save_yaml_) detected_markers_[ids[i]] = ps.pose;
      }
    }

    // Publish rectified images
    auto stamp = this->now();
    auto msgL = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left).toImageMsg();
    msgL->header.stamp = stamp; msgL->header.frame_id = "stereo_left_camera_frame";
    pub_left_->publish(*msgL);

    auto msgR = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right).toImageMsg();
    msgR->header.stamp = stamp; msgR->header.frame_id = "stereo_right_camera_frame";
    pub_right_->publish(*msgR);

    // Publish depth (Z) 32FC1 (m). Lấy Z từ points3D.
    cv::Mat depth(h, w, CV_32FC1);
    for (int y=0; y<h; ++y) {
      const cv::Vec3f* p3 = points3D.ptr<cv::Vec3f>(y);
      float* pd = depth.ptr<float>(y);
      for (int x=0; x<w; ++x) {
        float Z = p3[x][2];
        pd[x] = std::isfinite(Z) ? Z : std::numeric_limits<float>::quiet_NaN();
      }
    }
    auto msgD = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth).toImageMsg();
    msgD->header.stamp = stamp; msgD->header.frame_id = "stereo_left_camera_frame";
    pub_depth_->publish(*msgD);

    // Publish debug ArUco image
    if (pub_debug_->get_subscription_count() > 0) {
      auto dbg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left).toImageMsg();
      dbg->header.stamp = stamp; dbg->header.frame_id = "stereo_left_camera_frame";
      pub_debug_->publish(*dbg);
    }
  }

  void saveMapToYAML() {
    YAML::Node root;
    for (const auto& pair : detected_markers_) {
      int id = pair.first;
      const auto& pose = pair.second;
      YAML::Node marker_node;
      marker_node["position"]["x"] = pose.position.x;
      marker_node["position"]["y"] = pose.position.y;
      marker_node["position"]["z"] = pose.position.z;
      marker_node["orientation"]["x"] = pose.orientation.x;
      marker_node["orientation"]["y"] = pose.orientation.y;
      marker_node["orientation"]["z"] = pose.orientation.z;
      marker_node["orientation"]["w"] = pose.orientation.w;
      root["markers"][id] = marker_node;
    }

    std::ofstream fout(save_path_);
    fout << root;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoCpuVGA60Node>());
  rclcpp::shutdown();
  return 0;
}
