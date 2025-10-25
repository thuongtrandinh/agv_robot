#include "agv_fuzzy_trajectory/fuzzy.h"
#include <algorithm>

namespace agv_fuzzy_trajectory
{

FuzzyTrajectoryController::FuzzyTrajectoryController()
    : Node("fuzzy_trajectory_controller"),
      robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0),
      path_received_(false)
{
    // Declare parameters
    this->declare_parameter("wheel_base", 0.35);  // 350mm
    this->declare_parameter("wheel_radius", 0.065);  // 65mm
    this->declare_parameter("max_linear_vel", 0.5);  // m/s
    this->declare_parameter("max_angular_vel", 1.0);  // rad/s
    this->declare_parameter("control_frequency", 20.0);  // Hz
    this->declare_parameter("goal_tolerance", 0.1);  // meters

    // Get parameters
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    // Create subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&FuzzyTrajectoryController::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/trajectory", 10,
        std::bind(&FuzzyTrajectoryController::pathCallback, this, std::placeholders::_1));

    // Create publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create control timer
    auto timer_period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&FuzzyTrajectoryController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Fuzzy Trajectory Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Wheel base: %.3f m", wheel_base_);
    RCLCPP_INFO(this->get_logger(), "Control frequency: %.1f Hz", control_frequency_);
}

void FuzzyTrajectoryController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    // Convert quaternion to yaw
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, robot_theta_);
}

void FuzzyTrajectoryController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    current_path_ = *msg;
    path_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu points", 
                current_path_.poses.size());
}

// Triangular membership function
double FuzzyTrajectoryController::triangularMF(double x, double a, double b, double c)
{
    if (x <= a || x >= c)
        return 0.0;
    else if (x == b)
        return 1.0;
    else if (x > a && x < b)
        return (x - a) / (b - a);
    else // x > b && x < c
        return (c - x) / (c - b);
}

// Trapezoidal membership function
double FuzzyTrajectoryController::trapezoidalMF(double x, double a, double b, double c, double d)
{
    if (x <= a || x >= d)
        return 0.0;
    else if (x >= b && x <= c)
        return 1.0;
    else if (x > a && x < b)
        return (x - a) / (b - a);
    else // x > c && x < d
        return (d - x) / (d - c);
}

// Fuzzify lateral error (meters)
// Based on Omrane2016 Table 1 with added S level
std::vector<double> FuzzyTrajectoryController::fuzzifyLateralError(double e)
{
    std::vector<double> memberships(11, 0.0);
    
    // Convert to mm for easier interpretation
    double e_mm = e * 1000.0;
    
    // NVB: -inf to -100mm
    memberships[NVB] = trapezoidalMF(e_mm, -200, -150, -100, -80);
    
    // NB: -100 to -50mm
    memberships[NB] = triangularMF(e_mm, -100, -75, -50);
    
    // NM: -50 to -25mm
    memberships[NM] = triangularMF(e_mm, -50, -37.5, -25);
    
    // NS: -25 to -15mm
    memberships[NS] = triangularMF(e_mm, -25, -20, -15);
    
    // NZ: -15 to -5mm
    memberships[NZ] = triangularMF(e_mm, -15, -10, -5);
    
    // Z: -5 to 5mm
    memberships[Z] = triangularMF(e_mm, -5, 0, 5);
    
    // PZ: 5 to 15mm
    memberships[PZ] = triangularMF(e_mm, 5, 10, 15);
    
    // PS: 15 to 25mm (NEW LEVEL)
    memberships[PS] = triangularMF(e_mm, 15, 20, 25);
    
    // PM: 25 to 50mm
    memberships[PM] = triangularMF(e_mm, 25, 37.5, 50);
    
    // PB: 50 to 100mm
    memberships[PB] = triangularMF(e_mm, 50, 75, 100);
    
    // PVB: 100mm to +inf
    memberships[PVB] = trapezoidalMF(e_mm, 80, 100, 150, 200);
    
    return memberships;
}

// Fuzzify heading error (radians)
std::vector<double> FuzzyTrajectoryController::fuzzifyHeadingError(double theta)
{
    std::vector<double> memberships(11, 0.0);
    
    // Convert to degrees for easier interpretation
    double theta_deg = theta * 180.0 / M_PI;
    
    // Similar structure to lateral error
    memberships[NVB] = trapezoidalMF(theta_deg, -90, -60, -45, -35);
    memberships[NB] = triangularMF(theta_deg, -45, -30, -20);
    memberships[NM] = triangularMF(theta_deg, -20, -12.5, -7.5);
    memberships[NS] = triangularMF(theta_deg, -7.5, -5, -2.5);
    memberships[NZ] = triangularMF(theta_deg, -2.5, -1.25, 0);
    memberships[Z] = triangularMF(theta_deg, -1.25, 0, 1.25);
    memberships[PZ] = triangularMF(theta_deg, 0, 1.25, 2.5);
    memberships[PS] = triangularMF(theta_deg, 2.5, 5, 7.5);
    memberships[PM] = triangularMF(theta_deg, 7.5, 12.5, 20);
    memberships[PB] = triangularMF(theta_deg, 20, 30, 45);
    memberships[PVB] = trapezoidalMF(theta_deg, 35, 45, 60, 90);
    
    return memberships;
}

// Fuzzy inference engine (Mamdani type)
std::pair<double, double> FuzzyTrajectoryController::fuzzyInference(
    double lateral_error, double heading_error)
{
    // Fuzzify inputs
    auto e_mf = fuzzifyLateralError(lateral_error);
    auto theta_mf = fuzzifyHeadingError(heading_error);
    
    // Output membership values for V_left and V_right (mm/s)
    // Based on Omrane2016 fuzzy rule base
    std::vector<double> vl_output(11, 0.0);
    std::vector<double> vr_output(11, 0.0);
    
    // Output fuzzy sets (mm/s)
    // NVB=-200, NB=-150, NM=-100, NS=-50, NZ=-15, Z=0, PZ=15, PS=15, PM=50, PB=100, PVB=200
    std::vector<double> output_values = {-200, -150, -100, -50, -15, 0, 15, 15, 50, 100, 200};
    output_values[PS] = 15.0;  // NEW: PS = 15mm/s
    
    // Apply fuzzy rules (simplified rule base from Omrane2016)
    // Rule format: IF e is X AND theta is Y THEN V_L is Z1 AND V_R is Z2
    
    for (size_t i = 0; i < e_mf.size(); ++i)
    {
        for (size_t j = 0; j < theta_mf.size(); ++j)
        {
            double rule_strength = std::min(e_mf[i], theta_mf[j]);
            
            if (rule_strength > 0.0)
            {
                // Simplified rule base
                int vl_idx, vr_idx;
                
                // If error is zero and heading is zero -> go straight
                if (i == Z && j == Z) {
                    vl_idx = PM; vr_idx = PM;
                }
                // If lateral error is positive -> turn right (VL > VR)
                else if (i > Z) {
                    vl_idx = std::min((int)PVB, (int)(PM + (i - Z)));
                    vr_idx = std::max((int)PS, (int)(PM - (i - Z)));
                }
                // If lateral error is negative -> turn left (VR > VL)
                else {
                    vl_idx = std::max((int)PS, (int)(PM - (Z - i)));
                    vr_idx = std::min((int)PVB, (int)(PM + (Z - i)));
                }
                
                // Adjust based on heading error
                if (j > Z) { // heading error positive -> need to turn right
                    vl_idx = std::min((int)PVB, vl_idx + 1);
                    vr_idx = std::max((int)Z, vr_idx - 1);
                } else if (j < Z) { // heading error negative -> need to turn left
                    vl_idx = std::max((int)Z, vl_idx - 1);
                    vr_idx = std::min((int)PVB, vr_idx + 1);
                }
                
                vl_output[vl_idx] = std::max(vl_output[vl_idx], rule_strength);
                vr_output[vr_idx] = std::max(vr_output[vr_idx], rule_strength);
            }
        }
    }
    
    // Defuzzify using Center of Gravity
    double v_left = defuzzify(vl_output, output_values);
    double v_right = defuzzify(vr_output, output_values);
    
    return {v_left / 1000.0, v_right / 1000.0};  // Convert mm/s to m/s
}

// Defuzzification using Center of Gravity method
double FuzzyTrajectoryController::defuzzify(
    const std::vector<double>& memberships,
    const std::vector<double>& values)
{
    double numerator = 0.0;
    double denominator = 0.0;
    
    for (size_t i = 0; i < memberships.size(); ++i)
    {
        numerator += memberships[i] * values[i];
        denominator += memberships[i];
    }
    
    if (denominator < 1e-6)
        return 0.0;
    
    return numerator / denominator;
}

// Find closest point on trajectory
size_t FuzzyTrajectoryController::findClosestPoint(
    const nav_msgs::msg::Path& path, double x, double y)
{
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        double dx = path.poses[i].pose.position.x - x;
        double dy = path.poses[i].pose.position.y - y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_idx = i;
        }
    }
    
    return closest_idx;
}

// Compute tracking errors
void FuzzyTrajectoryController::computeErrors(double& lateral_error, double& heading_error)
{
    if (!path_received_ || current_path_.poses.empty())
    {
        lateral_error = 0.0;
        heading_error = 0.0;
        return;
    }
    
    // Find closest point on trajectory
    size_t closest_idx = findClosestPoint(current_path_, robot_x_, robot_y_);
    
    // Get reference point
    auto& ref_pose = current_path_.poses[closest_idx].pose;
    double ref_x = ref_pose.position.x;
    double ref_y = ref_pose.position.y;
    
    // Compute reference heading
    tf2::Quaternion q_ref(
        ref_pose.orientation.x,
        ref_pose.orientation.y,
        ref_pose.orientation.z,
        ref_pose.orientation.w);
    tf2::Matrix3x3 m_ref(q_ref);
    double roll_ref, pitch_ref, yaw_ref;
    m_ref.getRPY(roll_ref, pitch_ref, yaw_ref);
    
    // Compute lateral error (signed distance to trajectory)
    double dx = robot_x_ - ref_x;
    double dy = robot_y_ - ref_y;
    lateral_error = -std::sin(yaw_ref) * dx + std::cos(yaw_ref) * dy;
    
    // Compute heading error
    heading_error = robot_theta_ - yaw_ref;
    
    // Normalize heading error to [-pi, pi]
    while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
}

// Convert wheel velocities to global velocities (v, ω)
geometry_msgs::msg::Twist FuzzyTrajectoryController::wheelToGlobalVelocity(
    double v_left, double v_right)
{
    geometry_msgs::msg::Twist cmd_vel;
    
    // Kinematic model for differential drive
    cmd_vel.linear.x = (v_left + v_right) / 2.0;
    cmd_vel.angular.z = (v_right - v_left) / wheel_base_;
    
    // Apply velocity limits
    cmd_vel.linear.x = std::clamp(cmd_vel.linear.x, -max_linear_vel_, max_linear_vel_);
    cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -max_angular_vel_, max_angular_vel_);
    
    return cmd_vel;
}

// Main control loop
void FuzzyTrajectoryController::controlLoop()
{
    if (!path_received_ || current_path_.poses.empty())
        return;
    
    // Check if goal reached
    auto& goal = current_path_.poses.back().pose.position;
    double dist_to_goal = std::sqrt(
        std::pow(robot_x_ - goal.x, 2) + 
        std::pow(robot_y_ - goal.y, 2));
    
    if (dist_to_goal < goal_tolerance_)
    {
        // Stop the robot
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Goal reached!");
        return;
    }
    
    // Compute tracking errors
    double lateral_error, heading_error;
    computeErrors(lateral_error, heading_error);
    
    // Apply fuzzy logic controller
    auto [v_left, v_right] = fuzzyInference(lateral_error, heading_error);
    
    // Convert to global velocities and publish
    auto cmd_vel = wheelToGlobalVelocity(v_left, v_right);
    cmd_vel_pub_->publish(cmd_vel);
    
    // Debug output
    RCLCPP_DEBUG(this->get_logger(), 
                 "e=%.3f m, θ=%.3f rad, VL=%.3f m/s, VR=%.3f m/s, v=%.3f m/s, ω=%.3f rad/s",
                 lateral_error, heading_error, v_left, v_right, 
                 cmd_vel.linear.x, cmd_vel.angular.z);
}

} // namespace agv_fuzzy_trajectory

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<agv_fuzzy_trajectory::FuzzyTrajectoryController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
