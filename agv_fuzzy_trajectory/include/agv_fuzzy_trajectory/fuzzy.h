#ifndef AGV_FUZZY_TRAJECTORY_FUZZY_H
#define AGV_FUZZY_TRAJECTORY_FUZZY_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

namespace agv_fuzzy_trajectory
{

// Input fuzzy levels for Distance (D)
enum DistanceLevel
{
    VS = 0,   // Very Small
    S,        // Small
    M,        // Medium
    B,        // Big
    VB        // Very Big
};

// Input fuzzy levels for Angle (φ)
enum AngleLevel
{
    NB = 0,   // Negative Big
    NM,       // Negative Medium
    NS,       // Negative Small
    Z,        // Zero
    PS,       // Positive Small
    PM,       // Positive Medium
    PB        // Positive Big
};

// Output fuzzy levels for VL and VR
enum VelocityLevel
{
    V_Z = 0,  // Zero (0 mm/s)
    V_S,      // Small (15 mm/s) - NEW ADDED
    V_F,      // Forward (30 mm/s)
    V_M,      // Medium (40 mm/s)
    V_B,      // Big (50 mm/s)
    V_VB      // Very Big (70 mm/s)
};

class FuzzyTrajectoryController : public rclcpp::Node
{
public:
    FuzzyTrajectoryController();
    ~FuzzyTrajectoryController() = default;

private:
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void controlLoop();

    // Fuzzy logic functions
    double triangularMF(double x, double a, double b, double c);
    double trapezoidalMF(double x, double a, double b, double c, double d);
    
    // Fuzzification (based on Omrane2016 Figures 5 & 6)
    std::vector<double> fuzzifyDistance(double d);    // Distance in mm
    std::vector<double> fuzzifyAngle(double phi);     // Angle in degrees
    
    // Inference engine with fuzzy rule base from Table 1
    std::pair<double, double> fuzzyInference(double distance, double angle);
    
    // Defuzzification (Center of Gravity method)
    double defuzzify(const std::vector<double>& memberships, const std::vector<double>& singletons);
    
    // Trajectory tracking utilities
    size_t findClosestPoint(const nav_msgs::msg::Path& path, double x, double y);
    void computeErrors(double& lateral_error, double& heading_error);
    
    // Convert wheel velocities to global velocities (v, ω)
    geometry_msgs::msg::Twist wheelToGlobalVelocity(double v_left, double v_right);

    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Robot state
    double robot_x_, robot_y_, robot_theta_;
    nav_msgs::msg::Path current_path_;
    bool path_received_;

    // Robot parameters
    double wheel_base_;  // Distance between wheels (L)
    double wheel_radius_;
    double max_linear_vel_;
    double max_angular_vel_;
    
    // Control parameters
    double control_frequency_;
    double goal_tolerance_;
};

} // namespace agv_fuzzy_trajectory

#endif // AGV_FUZZY_TRAJECTORY_FUZZY_H
