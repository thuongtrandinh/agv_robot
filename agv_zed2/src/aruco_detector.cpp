// ============================================================================
// aruco_detector.cpp  (ZED2 Stereo 1344×376 @ 60 FPS with Triangulation)
// ============================================================================
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
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
        device_ = declare_parameter("device", "/dev/video0");
        frame_id_ = declare_parameter("camera_frame", "zed2_left_camera_frame");
        use_stereo_ = declare_parameter("use_stereo", true);
        target_fps_ = declare_parameter("target_fps", 30);  // Giảm xuống 30 FPS cho IPC

        // Setup ArUco detector parameters (tối ưu cho IPC)
        params_ = cv::aruco::DetectorParameters::create();
        params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        params_->cornerRefinementWinSize = 5;  // Giữ nhỏ để nhanh
        params_->cornerRefinementMaxIterations = 20;  // Giảm từ 30 → 20
        params_->cornerRefinementMinAccuracy = 0.02;  // Giảm chính xác một chút cho tốc độ
        params_->minCornerDistanceRate = 0.05;  // Tránh false positive
        
        // Board đơn giản cho refine (markerSeparation phải > 0)
        board_ = cv::aruco::GridBoard::create(1, 1, marker_size_, marker_size_ * 0.02f, dict_);

        loadCalibration();

        // Open left camera (ZED2 on IPC: /dev/video0 = left, /dev/video1 = right)
        cap_left_.open(device_, cv::CAP_V4L2);
        if (!cap_left_.isOpened()) {
            RCLCPP_FATAL(get_logger(), "❌ Cannot open left camera %s", device_.c_str());
            RCLCPP_FATAL(get_logger(), "💡 Tip: Check if device is busy with: sudo fuser -v %s", device_.c_str());
            rclcpp::shutdown();
            return;
        }

        // Set format for left camera
        cap_left_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
        cap_left_.set(cv::CAP_PROP_FRAME_WIDTH, img_width_);   // 672
        cap_left_.set(cv::CAP_PROP_FRAME_HEIGHT, img_height_); // 376
        cap_left_.set(cv::CAP_PROP_FPS, target_fps_);
        cap_left_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        int actual_w_l = cap_left_.get(cv::CAP_PROP_FRAME_WIDTH);
        int actual_h_l = cap_left_.get(cv::CAP_PROP_FRAME_HEIGHT);
        int actual_fps_l = cap_left_.get(cv::CAP_PROP_FPS);
        
        RCLCPP_INFO(get_logger(), "📸 Left camera opened: %dx%d @ %d FPS", 
                    actual_w_l, actual_h_l, actual_fps_l);

        // Open right camera if stereo mode
        if (use_stereo_) {
            std::string right_device = "/dev/video1";  // Right camera always video1
            cap_right_.open(right_device, cv::CAP_V4L2);
            if (!cap_right_.isOpened()) {
                RCLCPP_WARN(get_logger(), "⚠️  Cannot open right camera %s, using mono mode", right_device.c_str());
                RCLCPP_WARN(get_logger(), "💡 Tip: Check if device is busy with: sudo fuser -v %s", right_device.c_str());
                use_stereo_ = false;
            } else {
                // Set format for right camera
                cap_right_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
                cap_right_.set(cv::CAP_PROP_FRAME_WIDTH, img_width_);
                cap_right_.set(cv::CAP_PROP_FRAME_HEIGHT, img_height_);
                cap_right_.set(cv::CAP_PROP_FPS, target_fps_);
                cap_right_.set(cv::CAP_PROP_BUFFERSIZE, 1);

                int actual_w_r = cap_right_.get(cv::CAP_PROP_FRAME_WIDTH);
                int actual_h_r = cap_right_.get(cv::CAP_PROP_FRAME_HEIGHT);
                int actual_fps_r = cap_right_.get(cv::CAP_PROP_FPS);
                
                RCLCPP_INFO(get_logger(), "📸 Right camera opened: %dx%d @ %d FPS", 
                            actual_w_r, actual_h_r, actual_fps_r);
            }
        }
        
        RCLCPP_INFO(get_logger(), "✅ ZED2 ready (stereo mode: %s)", use_stereo_ ? "ON" : "OFF");

        pub_detections_ = create_publisher<std_msgs::msg::Float32MultiArray>("/aruco/detections", 10);

        // 30 FPS = 33ms per frame (đủ thời gian xử lý cho IPC)
        auto period = std::chrono::milliseconds(1000 / target_fps_);
        timer_ = create_wall_timer(
            period,
            std::bind(&ArucoDetectorNode::processFrame, this)
        );
        
        RCLCPP_INFO(get_logger(), "⚙️ Optimized for IPC: %d FPS, low-overhead mode", target_fps_);
    }

private:
    void loadCalibration() {
        std::string pkg = ament_index_cpp::get_package_share_directory(calib_pkg_);
        std::string path = pkg + "/config/" + calib_file_;

        YAML::Node yaml = YAML::LoadFile(path);
        auto camL = yaml["left_camera"];
        auto camR = yaml["right_camera"];

        // Left camera intrinsics
        auto KdataL = camL["camera_matrix"]["data"].as<std::vector<double>>();
        auto DdataL = camL["distortion_coefficients"]["data"].as<std::vector<double>>();
        K_left_ = cv::Mat(3,3,CV_64F);
        for (int i=0;i<9;i++) K_left_.at<double>(i/3,i%3) = KdataL[i];
        D_left_ = cv::Mat(1,5,CV_64F);
        for (int i=0;i<5;i++) D_left_.at<double>(0,i) = DdataL[i];

        // Right camera intrinsics
        auto KdataR = camR["camera_matrix"]["data"].as<std::vector<double>>();
        auto DdataR = camR["distortion_coefficients"]["data"].as<std::vector<double>>();
        K_right_ = cv::Mat(3,3,CV_64F);
        for (int i=0;i<9;i++) K_right_.at<double>(i/3,i%3) = KdataR[i];
        D_right_ = cv::Mat(1,5,CV_64F);
        for (int i=0;i<5;i++) D_right_.at<double>(0,i) = DdataR[i];

        // Image dimensions from YAML
        img_height_ = camL["image_height"].as<int>();
        img_width_ = camL["image_width"].as<int>();
        stereo_width_ = img_width_ * 2; // Side-by-side: 1344 = 672*2

        // Extract stereo extrinsics from projection matrix (ZED SDK format)
        // Right P matrix: [fx, 0, cx, Tx; 0, fy, cy, 0; 0, 0, 1, 0]
        // where Tx = -fx * baseline
        auto PdataR = camR["projection_matrix"]["data"].as<std::vector<double>>();
        double Tx = PdataR[3];  // Tx = -32.0 for ZED2
        double fx_right = PdataR[0];  // fx ≈ 266.954
        
        // Calculate baseline from Tx = -fx * baseline
        // baseline = -Tx / fx = 32.0 / 266.954 ≈ 0.1199m ≈ 120mm
        baseline_ = -Tx / fx_right;
        
        // For ZED2 raw stereo: R = I (identity), T = [-baseline, 0, 0]
        R_ = cv::Mat::eye(3, 3, CV_64F);
        T_ = (cv::Mat_<double>(3,1) << -baseline_, 0.0, 0.0);

        // For backward compatibility
        K_ = K_left_;
        D_ = D_left_;

        RCLCPP_INFO(get_logger(), "📂 Loaded ZED2 VGA calibration: %s", path.c_str());
        RCLCPP_INFO(get_logger(), "   Resolution: %dx%d (stereo: %dx%d)", 
                    img_width_, img_height_, stereo_width_, img_height_);
        RCLCPP_INFO(get_logger(), "   Baseline: %.1f mm (Tx: %.1f)", baseline_*1000, Tx);

        // Compute rectification maps (nếu dùng stereo)
        if (use_stereo_) {
            cv::stereoRectify(K_left_, D_left_, K_right_, D_right_, 
                            cv::Size(img_width_, img_height_), R_, T_,
                            R1_, R2_, P1_, P2_, Q_, 
                            cv::CALIB_ZERO_DISPARITY, -1, 
                            cv::Size(img_width_, img_height_));

            cv::initUndistortRectifyMap(K_left_, D_left_, R1_, P1_, 
                                      cv::Size(img_width_, img_height_), 
                                      CV_32FC1, map1x_, map1y_);
            cv::initUndistortRectifyMap(K_right_, D_right_, R2_, P2_, 
                                      cv::Size(img_width_, img_height_), 
                                      CV_32FC1, map2x_, map2y_);
            
            RCLCPP_INFO(get_logger(), "   ✓ Stereo rectification maps computed");
        }
    }

    void processFrame() {
        // Capture from left camera
        cv::Mat img_left;
        cap_left_ >> img_left;
        if (img_left.empty()) return;

        // Convert YUYV to grayscale directly (tránh BGR intermediate)
        cv::Mat left_gray_raw;
        cv::cvtColor(img_left, left_gray_raw, cv::COLOR_YUV2GRAY_YUYV);

        if (use_stereo_) {
            // Capture from right camera
            cv::Mat img_right;
            cap_right_ >> img_right;
            if (img_right.empty()) {
                // Right camera failed, fallback to mono
                cv::Mat left_gray;
                cv::remap(left_gray_raw, left_gray, map1x_, map1y_, cv::INTER_LINEAR);
                detectAruco(left_gray);
                return;
            }

            cv::Mat right_gray_raw;
            cv::cvtColor(img_right, right_gray_raw, cv::COLOR_YUV2GRAY_YUYV);

            // Rectify both images
            cv::Mat left_gray, right_gray;
            cv::remap(left_gray_raw, left_gray, map1x_, map1y_, cv::INTER_LINEAR);
            cv::remap(right_gray_raw, right_gray, map2x_, map2y_, cv::INTER_LINEAR);

            // Detect ArUco on both views for triangulation
            detectArucoStereo(left_gray, right_gray);
        }
        else {
            // Mono mode: use left camera only
            cv::Mat left_gray;
            cv::remap(left_gray_raw, left_gray, map1x_, map1y_, cv::INTER_LINEAR);
            detectAruco(left_gray);
        }
    }

    void detectAruco(const cv::Mat &img) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        cv::aruco::detectMarkers(img, dict_, corners, ids, params_, rejected);
        if (ids.empty()) return;

        // Refine detection (chỉ khi cần thiết để giảm CPU)
        if (corners.size() < 5) {  // Chỉ refine khi ít marker
            cv::aruco::refineDetectedMarkers(img, board_, corners, ids, rejected, K_left_, D_left_);
        }

        // Use K_left_ (intrinsic gốc) sau khi rectified, distortion = 0
        cv::Mat D_zero = cv::Mat::zeros(1, 5, CV_64F);

        // Vectorized: tính tất cả marker một lần
        std::vector<cv::Vec3d> rvecs, tvecs;
        if (!corners.empty()) {
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, K_left_, D_zero, rvecs, tvecs);
        }

        // Không cần filter duplicate nếu detection tốt, giảm overhead
        std_msgs::msg::Float32MultiArray detections_msg;
        detections_msg.data.reserve(ids.size() * 8);  // Pre-allocate để tránh realloc
        
        for (size_t i = 0; i < ids.size(); i++) {
            cv::Mat R_cv;
            cv::Rodrigues(rvecs[i], R_cv);

            // ZED left camera frame = OpenCV camera frame
            // x→right, y→down, z→forward (chuẩn camera, không cần xoay)
            cv::Mat tvec_mat = cv::Mat(tvecs[i]);

            tf2::Matrix3x3 m(
                R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
                R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
                R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2)
            );

            tf2::Quaternion q;
            m.getRotation(q);

            detections_msg.data.push_back(static_cast<float>(ids[i]));
            detections_msg.data.push_back(static_cast<float>(tvec_mat.at<double>(0)));
            detections_msg.data.push_back(static_cast<float>(tvec_mat.at<double>(1)));
            detections_msg.data.push_back(static_cast<float>(tvec_mat.at<double>(2)));
            detections_msg.data.push_back(static_cast<float>(q.x()));
            detections_msg.data.push_back(static_cast<float>(q.y()));
            detections_msg.data.push_back(static_cast<float>(q.z()));
            detections_msg.data.push_back(static_cast<float>(q.w()));
        }

        pub_detections_->publish(detections_msg);
    }

    void detectArucoStereo(const cv::Mat &left_gray, const cv::Mat &right_gray) {
        // Detect markers in left image
        std::vector<int> ids_left, ids_right;
        std::vector<std::vector<cv::Point2f>> corners_left, corners_right, rejected_left, rejected_right;
        
        cv::aruco::detectMarkers(left_gray, dict_, corners_left, ids_left, params_, rejected_left);
        if (ids_left.empty()) return;

        // Detect in right image (parallel với left để tiết kiệm thời gian)
        cv::aruco::detectMarkers(right_gray, dict_, corners_right, ids_right, params_, rejected_right);

        // Refine chỉ khi ít marker (giảm CPU load)
        if (corners_left.size() < 5) {
            cv::aruco::refineDetectedMarkers(left_gray, board_, corners_left, ids_left, rejected_left, K_left_, D_left_);
            cv::aruco::refineDetectedMarkers(right_gray, board_, corners_right, ids_right, rejected_right, K_right_, D_right_);
        }

        cv::Mat D_zero = cv::Mat::zeros(1, 5, CV_64F);
        std_msgs::msg::Float32MultiArray detections_msg;
        detections_msg.data.reserve(ids_left.size() * 8);

        // Vectorized orientation estimation một lần cho tất cả markers
        std::vector<cv::Vec3d> rvecs_all, tvecs_all;
        if (!corners_left.empty()) {
            cv::aruco::estimatePoseSingleMarkers(corners_left, marker_size_, K_left_, D_zero, rvecs_all, tvecs_all);
        }

        // Process each marker detected in left image
        for (size_t i = 0; i < ids_left.size(); i++) {
            int marker_id = ids_left[i];
            
            // Find corresponding marker in right image
            auto it_right = std::find(ids_right.begin(), ids_right.end(), marker_id);
            
            cv::Vec3d tvec;
            cv::Vec3d rvec;
            bool have_stereo = false;

            if (it_right != ids_right.end()) {
                // Stereo match found - dùng cv::triangulatePoints cho 4 corners
                size_t idx_right = std::distance(ids_right.begin(), it_right);
                
                // Chuẩn bị 4 cặp điểm tương ứng
                cv::Mat ptsL(2, 4, CV_64F), ptsR(2, 4, CV_64F);
                for (int j = 0; j < 4; j++) {
                    ptsL.at<double>(0, j) = corners_left[i][j].x;
                    ptsL.at<double>(1, j) = corners_left[i][j].y;
                    ptsR.at<double>(0, j) = corners_right[idx_right][j].x;
                    ptsR.at<double>(1, j) = corners_right[idx_right][j].y;
                }

                // Triangulate 4 corners
                cv::Mat X;
                cv::triangulatePoints(P1_, P2_, ptsL, ptsR, X);

                // Average 4 3D points để có center
                cv::Vec3d center_3d(0, 0, 0);
                for (int j = 0; j < 4; j++) {
                    double w = X.at<double>(3, j);
                    center_3d[0] += X.at<double>(0, j) / w;
                    center_3d[1] += X.at<double>(1, j) / w;
                    center_3d[2] += X.at<double>(2, j) / w;
                }
                center_3d *= 0.25;

                // Kiểm tra disparity hợp lệ (Z > 0, disparity > 0)
                cv::Point2f center_left(0, 0), center_right(0, 0);
                for (const auto& pt : corners_left[i]) center_left += pt;
                for (const auto& pt : corners_right[idx_right]) center_right += pt;
                center_left *= 0.25f;
                center_right *= 0.25f;
                
                float disparity = center_left.x - center_right.x;
                
                if (disparity > 1.0f && center_3d[2] > 0.1) {  // Valid depth
                    tvec = center_3d;
                    have_stereo = true;
                }
            }
            
            // Get orientation from vectorized result (đã tính sẵn)
            rvec = rvecs_all[i];
            
            // If stereo failed, use single-view depth từ vectorized result
            if (!have_stereo) {
                tvec = tvecs_all[i];
            }

            // Không xoay frame - ZED left camera frame = OpenCV frame
            cv::Mat R_cv;
            cv::Rodrigues(rvec, R_cv);
            cv::Mat tvec_mat = cv::Mat(tvec);

            tf2::Matrix3x3 m(
                R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
                R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
                R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2)
            );

            tf2::Quaternion q;
            m.getRotation(q);

            detections_msg.data.push_back(static_cast<float>(marker_id));
            detections_msg.data.push_back(static_cast<float>(tvec_mat.at<double>(0)));
            detections_msg.data.push_back(static_cast<float>(tvec_mat.at<double>(1)));
            detections_msg.data.push_back(static_cast<float>(tvec_mat.at<double>(2)));
            detections_msg.data.push_back(static_cast<float>(q.x()));
            detections_msg.data.push_back(static_cast<float>(q.y()));
            detections_msg.data.push_back(static_cast<float>(q.z()));
            detections_msg.data.push_back(static_cast<float>(q.w()));
        }

        pub_detections_->publish(detections_msg);
    }

private:
    cv::Ptr<cv::aruco::Dictionary> dict_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;
    cv::Ptr<cv::aruco::GridBoard> board_;
    cv::Mat K_, D_;  // Keep for backward compatibility (will point to K_left_, D_left_)
    cv::VideoCapture cap_left_;   // Left camera /dev/video0
    cv::VideoCapture cap_right_;  // Right camera /dev/video1
    
    // Stereo camera parameters
    cv::Mat K_left_, D_left_, K_right_, D_right_;
    cv::Mat R_, T_;  // Stereo extrinsics
    cv::Mat R1_, R2_, P1_, P2_, Q_;  // Rectification matrices
    cv::Mat map1x_, map1y_, map2x_, map2y_;  // Undistortion maps
    int img_width_, img_height_, stereo_width_;
    double baseline_;  // Stereo baseline in meters
    bool use_stereo_;

    std::string calib_pkg_, calib_file_, device_, frame_id_;
    double marker_size_;
    int target_fps_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_detections_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
