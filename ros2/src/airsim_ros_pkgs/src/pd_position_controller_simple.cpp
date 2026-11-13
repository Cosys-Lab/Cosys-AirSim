#include "pd_position_controller_simple.h"
using namespace std::placeholders;

bool PIDParams::load_from_rosparams(const std::shared_ptr<rclcpp::Node> nh)
{
    bool found = true;

    found = found && nh->get_parameter("kp_x", kp_x);
    found = found && nh->get_parameter("kp_y", kp_y);
    found = found && nh->get_parameter("kp_z", kp_z);
    found = found && nh->get_parameter("kp_yaw", kp_yaw);

    found = found && nh->get_parameter("kd_x", kd_x);
    found = found && nh->get_parameter("kd_y", kd_y);
    found = found && nh->get_parameter("kd_z", kd_z);
    found = found && nh->get_parameter("kd_yaw", kd_yaw);

    found = found && nh->get_parameter("reached_thresh_xyz", reached_thresh_xyz);
    found = found && nh->get_parameter("reached_yaw_degrees", reached_yaw_degrees);

    return found;
}

bool DynamicConstraints::load_from_rosparams(const std::shared_ptr<rclcpp::Node> nh)
{
    bool found = true;

    found = found && nh->get_parameter("max_vel_horz_abs", max_vel_horz_abs);
    found = found && nh->get_parameter("max_vel_vert_abs", max_vel_vert_abs);
    found = found && nh->get_parameter("max_yaw_rate_degree", max_yaw_rate_degree);

    return found;
}

PIDPositionController::PIDPositionController(const std::shared_ptr<rclcpp::Node> nh)
    : use_eth_lib_for_geodetic_conv_(true), nh_(nh), has_home_geo_(false), reached_goal_(false), has_goal_(false), has_odom_(false), got_goal_once_(false)
{
    params_.load_from_rosparams(nh_);
    constraints_.load_from_rosparams(nh_);
    initialize_ros();
    reset_errors();
}

void PIDPositionController::reset_errors()
{
    prev_error_.x = 0.0;
    prev_error_.y = 0.0;
    prev_error_.z = 0.0;
    prev_error_.yaw = 0.0;
}

void PIDPositionController::initialize_ros()
{
    vel_cmd_ = airsim_interfaces::msg::VelCmd();
    // ROS params
    double update_control_every_n_sec = 0.01;
    nh_->get_parameter("update_control_every_n_sec", update_control_every_n_sec);

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(nh_, "/airsim_node");
    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(nh_->get_logger(), "service not available, waiting again...");
    }

    std::string vehicle_name;

    while (vehicle_name == "") {
        vehicle_name = parameters_client->get_parameter("vehicle_name", vehicle_name);
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Waiting vehicle name");
    }
    // the odometry topic is named after the odometry frame of airsim_node (default odom_local)
    const std::string odom_frame_id = parameters_client->get_parameter("odom_frame_id", std::string("odom_local"));
    const std::string topic_prefix = "/airsim_node/" + vehicle_name;

    // ROS publishers
    airsim_vel_cmd_world_frame_pub_ = nh_->create_publisher<airsim_interfaces::msg::VelCmd>(topic_prefix + "/vel_cmd_world_frame", 1);

    // ROS subscribers
    airsim_odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(topic_prefix + "/" + odom_frame_id, 50, std::bind(&PIDPositionController::airsim_odom_cb, this, _1));
    home_geopoint_sub_ = nh_->create_subscription<airsim_interfaces::msg::GPSYaw>("/airsim_node/origin_geo_point", 50, std::bind(&PIDPositionController::home_geopoint_cb, this, _1));
    local_position_goal_srvr_ = nh_->create_service<airsim_interfaces::srv::SetLocalPosition>(topic_prefix + "/local_position_goal", std::bind(&PIDPositionController::local_position_goal_srv_cb, this, _1, _2));
    local_position_goal_override_srvr_ = nh_->create_service<airsim_interfaces::srv::SetLocalPosition>(topic_prefix + "/local_position_goal/override", std::bind(&PIDPositionController::local_position_goal_srv_override_cb, this, _1, _2));
    gps_goal_srvr_ = nh_->create_service<airsim_interfaces::srv::SetGPSPosition>(topic_prefix + "/gps_goal", std::bind(&PIDPositionController::gps_goal_srv_cb, this, _1, _2));
    gps_goal_override_srvr_ = nh_->create_service<airsim_interfaces::srv::SetGPSPosition>(topic_prefix + "/gps_goal/override", std::bind(&PIDPositionController::gps_goal_srv_override_cb, this, _1, _2));

    // ROS timers
    update_control_cmd_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_control_every_n_sec), std::bind(&PIDPositionController::update_control_cmd_timer_cb, this));
}

void PIDPositionController::airsim_odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    has_odom_ = true;
    curr_odom_ = *odom_msg;
    curr_position_.x = odom_msg->pose.pose.position.x;
    curr_position_.y = odom_msg->pose.pose.position.y;
    curr_position_.z = odom_msg->pose.pose.position.z;
    curr_position_.yaw = utils::get_yaw_from_quat_msg(odom_msg->pose.pose.orientation);
}

// todo maintain internal representation as eigen vec?
// todo check if low velocity if within thresh?
// todo maintain separate errors for XY and Z
void PIDPositionController::check_reached_goal()
{
    double diff_xyz = sqrt((target_position_.x - curr_position_.x) * (target_position_.x - curr_position_.x) + (target_position_.y - curr_position_.y) * (target_position_.y - curr_position_.y) + (target_position_.z - curr_position_.z) * (target_position_.z - curr_position_.z));

    double diff_yaw = std::fabs(math_common::angular_dist(target_position_.yaw, curr_position_.yaw));

    // todo save this in degrees somewhere to avoid repeated conversion
    if (diff_xyz < params_.reached_thresh_xyz && diff_yaw < math_common::deg2rad(params_.reached_yaw_degrees))
        reached_goal_ = true;
}

bool PIDPositionController::local_position_goal_srv_cb(const std::shared_ptr<airsim_interfaces::srv::SetLocalPosition::Request> request, std::shared_ptr<airsim_interfaces::srv::SetLocalPosition::Response> response)
{
    // this tells the update timer callback to not do active hovering
    if (!got_goal_once_)
        got_goal_once_ = true;

    if (has_goal_ && !reached_goal_) {
        // todo maintain array of position goals
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "denying position goal request-> I am still following the previous goal");
        response->success = false;
        response->message = "still following the previous goal";
        return false;
    }

    if (!has_goal_) {
        target_position_.x = request->x;
        target_position_.y = request->y;
        target_position_.z = request->z;
        target_position_.yaw = request->yaw;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "got goal: x=" << target_position_.x << " y=" << target_position_.y << " z=" << target_position_.z << " yaw=" << target_position_.yaw);

        has_goal_ = true;
        reached_goal_ = false;
        reset_errors();
        response->success = true;
        return true;
    }

    // Already have goal, and have reached it
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Already have goal and have reached it");
    response->success = false;
    response->message = "already have a goal";
    return false;
}

bool PIDPositionController::local_position_goal_srv_override_cb(const std::shared_ptr<airsim_interfaces::srv::SetLocalPosition::Request> request, std::shared_ptr<airsim_interfaces::srv::SetLocalPosition::Response> response)
{
    // this tells the update timer callback to not do active hovering
    if (!got_goal_once_)
        got_goal_once_ = true;

    target_position_.x = request->x;
    target_position_.y = request->y;
    target_position_.z = request->z;
    target_position_.yaw = request->yaw;
    RCLCPP_INFO_STREAM(nh_->get_logger(), "got goal: x=" << target_position_.x << " y=" << target_position_.y << " z=" << target_position_.z << " yaw=" << target_position_.yaw);

    has_goal_ = true;
    reached_goal_ = false;
    reset_errors();
    response->success = true;
    return true;
}

void PIDPositionController::home_geopoint_cb(const airsim_interfaces::msg::GPSYaw::SharedPtr gps_msg)
{
    if (has_home_geo_)
        return;
    gps_home_msg_ = *gps_msg;
    has_home_geo_ = true;
    RCLCPP_INFO_STREAM(nh_->get_logger(), "GPS reference initializing " << gps_msg->latitude << ", " << gps_msg->longitude << ", " << gps_msg->altitude);
    geodetic_converter_.initialiseReference(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
}

// Converts a GPS goal (absolute altitude) to the odometry frame of airsim_node: x north, y west, z up from the origin geo
// point, which is the take-off point when the vehicle starts at the PlayerStart. The yaw uses the same convention.
void PIDPositionController::set_target_from_gps(const airsim_interfaces::srv::SetGPSPosition::Request& request)
{
    double n, e, d;
    if (use_eth_lib_for_geodetic_conv_) {
        geodetic_converter_.geodetic2Ned(request.latitude, request.longitude, request.altitude, &n, &e, &d);
    }
    else // use airlib::GeodeticToNedFast
    {
        msr::airlib::GeoPoint goal_gps_point(request.latitude, request.longitude, request.altitude);
        msr::airlib::GeoPoint gps_home(gps_home_msg_.latitude, gps_home_msg_.longitude, gps_home_msg_.altitude);
        msr::airlib::Vector3r ned_goal = msr::airlib::EarthUtils::GeodeticToNedFast(goal_gps_point, gps_home);
        n = ned_goal[0];
        e = ned_goal[1];
        d = ned_goal[2];
    }
    target_position_.x = n;
    target_position_.y = -e;
    target_position_.z = -d;
    target_position_.yaw = request.yaw;
    RCLCPP_INFO_STREAM(nh_->get_logger(), "got GPS goal: lat=" << request.latitude << " long=" << request.longitude << " alt=" << request.altitude << " yaw=" << request.yaw);
    RCLCPP_INFO_STREAM(nh_->get_logger(), "converted local goal is: x=" << target_position_.x << " y=" << target_position_.y << " z=" << target_position_.z << " yaw=" << target_position_.yaw);

    has_goal_ = true;
    reached_goal_ = false;
    reset_errors();
}

bool PIDPositionController::gps_goal_srv_cb(const std::shared_ptr<airsim_interfaces::srv::SetGPSPosition::Request> request, std::shared_ptr<airsim_interfaces::srv::SetGPSPosition::Response> response)
{
    if (!has_home_geo_) {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "I don't have home GPS coord. Can't go to GPS goal!");
        response->success = false;
        return false;
    }
    if (has_goal_) {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Goal already received, ignoring!");
        response->success = false;
        return false;
    }
    if (!got_goal_once_)
        got_goal_once_ = true;
    set_target_from_gps(*request);
    response->success = true;
    return true;
}

bool PIDPositionController::gps_goal_srv_override_cb(const std::shared_ptr<airsim_interfaces::srv::SetGPSPosition::Request> request, std::shared_ptr<airsim_interfaces::srv::SetGPSPosition::Response> response)
{
    if (!has_home_geo_) {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "I don't have home GPS coord. Can't go to GPS goal!");
        response->success = false;
        return false;
    }
    if (!got_goal_once_)
        got_goal_once_ = true;
    set_target_from_gps(*request);
    response->success = true;
    return true;
}

void PIDPositionController::update_control_cmd_timer_cb()
{
    // todo check if odometry is too old!!
    // if no odom, don't do anything.
    if (!has_odom_) {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Waiting for odometry!");
        return;
    }

    if (has_goal_) {
        check_reached_goal();
        if (reached_goal_) {
            RCLCPP_INFO_STREAM(nh_->get_logger(), "Reached goal! Hovering at position.");
            has_goal_ = false;
            // dear future self, this function doesn't return coz we need to keep on actively hovering at last goal pose. don't act smart
        }
        else {
            RCLCPP_INFO_STREAM(nh_->get_logger(), "Moving to goal.");
        }
    }

    // only compute and send control commands for hovering / moving to pose, if we currently have a goal
    if (has_goal_) {
        compute_control_cmd();
        enforce_dynamic_constraints();
        publish_control_cmd();
    }
}

void PIDPositionController::compute_control_cmd()
{
    curr_error_.x = target_position_.x - curr_position_.x;
    curr_error_.y = target_position_.y - curr_position_.y;
    curr_error_.z = target_position_.z - curr_position_.z;
    curr_error_.yaw = math_common::angular_dist(curr_position_.yaw, target_position_.yaw);

    double p_term_x = params_.kp_x * curr_error_.x;
    double p_term_y = params_.kp_y * curr_error_.y;
    double p_term_z = params_.kp_z * curr_error_.z;
    double p_term_yaw = params_.kp_yaw * curr_error_.yaw;

    // derivative of the error per control update
    double d_term_x = params_.kd_x * (curr_error_.x - prev_error_.x);
    double d_term_y = params_.kd_y * (curr_error_.y - prev_error_.y);
    double d_term_z = params_.kd_z * (curr_error_.z - prev_error_.z);
    double d_term_yaw = params_.kd_yaw * math_common::angular_dist(prev_error_.yaw, curr_error_.yaw);

    prev_error_ = curr_error_;

    // the error is in the odometry frame of airsim_node, which is also the frame of the world frame velocity command
    vel_cmd_.twist.linear.x = p_term_x + d_term_x;
    vel_cmd_.twist.linear.y = p_term_y + d_term_y;
    vel_cmd_.twist.linear.z = p_term_z + d_term_z;
    vel_cmd_.twist.angular.z = p_term_yaw + d_term_yaw;
}

void PIDPositionController::enforce_dynamic_constraints()
{
    double vel_norm_horz = sqrt((vel_cmd_.twist.linear.x * vel_cmd_.twist.linear.x) + (vel_cmd_.twist.linear.y * vel_cmd_.twist.linear.y));

    if (vel_norm_horz > constraints_.max_vel_horz_abs) {
        vel_cmd_.twist.linear.x = (vel_cmd_.twist.linear.x / vel_norm_horz) * constraints_.max_vel_horz_abs;
        vel_cmd_.twist.linear.y = (vel_cmd_.twist.linear.y / vel_norm_horz) * constraints_.max_vel_horz_abs;
    }

    if (std::fabs(vel_cmd_.twist.linear.z) > constraints_.max_vel_vert_abs) {
        // todo just add a sgn funciton in common utils? return double to be safe.
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        vel_cmd_.twist.linear.z = (vel_cmd_.twist.linear.z / std::fabs(vel_cmd_.twist.linear.z)) * constraints_.max_vel_vert_abs;
    }
    // yaw rate limit, the command is in radians per second
    const double max_yaw_rate = math_common::deg2rad(constraints_.max_yaw_rate_degree);
    if (std::fabs(vel_cmd_.twist.angular.z) > max_yaw_rate) {
        vel_cmd_.twist.angular.z = (vel_cmd_.twist.angular.z / std::fabs(vel_cmd_.twist.angular.z)) * max_yaw_rate;
    }
}

void PIDPositionController::publish_control_cmd()
{
    airsim_vel_cmd_world_frame_pub_->publish(vel_cmd_);
}
