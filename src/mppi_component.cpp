#include "mppi_planner/mppi_component.hpp"

namespace mppi
{
MPPI::MPPI(const rclcpp::NodeOptions & node_options)
: Node("mppi_planner",node_options),
  tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
  tf_listener_(tf_buffer_)
{
    setParameters();
    
    // publisher
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    mppi_optimal_traj_pub_ = create_publisher<nav_msgs::msg::Path>("optimal_trajectory", 10);
    mppi_sample_traj_pub_ = create_publisher<nav_msgs::msg::Path>("sampled_trajectories", 10);
    // subscriber
    ref_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "reference_path", rclcpp::QoS(10).transient_local().reliable().keep_last(1),
      std::bind(&MPPI::refPathCallback, this, std::placeholders::_1));

    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "local_costmap", rclcpp::QoS(10).transient_local().reliable().keep_last(1), 
      std::bind(&MPPI::CostmapCallback, this, std::placeholders::_1)
    );
    
    // timer
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&MPPI::controlLoop, this)
    ); 
}

MPPI::~MPPI(){}

void MPPI::setParameters()
{
    control_frequency_ = declare_parameter<double>("control_frequency", 20.0);
    base_frame_id_ =  declare_parameter<std::string>("base_frame_id", "base_footprint");
    global_frame_id_ = declare_parameter<std::string>("global_frame_id", "map");
    enable_optical_paths_visualization_ = declare_parameter<bool>("enable_optical_paths_visualization", true);
    enable_sampled_paths_visualization_ = declare_parameter<bool>("enable_sampled_paths_visualization", false);
    max_sampled_paths_display_ = declare_parameter<int>("max_sampled_paths_display", 50);
    
    // MPPI algorithm parameters
    MPPIParams mppi_params_;
    mppi_params_.T = declare_parameter<int>("horizon_step", 30);
    mppi_params_.K = declare_parameter<int>("number_of_samples", 100);
    mppi_params_.UDIM = 2;
    mppi_params_.XDIM = 3;
    mppi_params_.dt = declare_parameter<double>("dt", 0.1);
    mppi_params_.lambda = declare_parameter<double>("lambda", 50.0);
    mppi_params_.alpha = declare_parameter<double>("alpha", 1.0);
    mppi_params_.exploration_rate = declare_parameter<double>("exploration_rate", 0.1);
    mppi_params_.sigma = {declare_parameter<double>("sigma_v", 0.5), declare_parameter<double>("sigma_w", 0.3)};
    mppi_params_.max_linear_velocity = declare_parameter<double>("max_linear_velocity", 1.0);
    mppi_params_.max_angular_velocity = declare_parameter<double>("max_angular_velocity", 0.5);
    
    try {
        mppi_controller_ = std::make_shared<mppi_core::MPPICore>(mppi_params_);
        RCLCPP_INFO(get_logger(), "MPPI controller initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize MPPI controller: %s", e.what());
        throw;
    }
    
    // Noise parameters
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.2);
}

void MPPI::refPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg || msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty or invalid path");
    ref_path_received_ = false;
    return;
  }
  
  ref_path_received_ = true;
  current_ref_path_ = *msg;
  closest_idx_ = 0;
  
  RCLCPP_INFO(get_logger(), "Received reference path with %zu poses", msg->poses.size());
}

void MPPI::getCurrentPose()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      global_frame_id_, base_frame_id_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO( 
      get_logger(), "Could not transform from %s to %s: %s", 
      base_frame_id_.c_str(), global_frame_id_.c_str(), ex.what());
    return;
  }
  current_pose_.resize(3);
  current_pose_[0] = transform.transform.translation.x;
  current_pose_[1] = transform.transform.translation.y;
  current_pose_[2] = tf2::getYaw(transform.transform.rotation);
  if (!pose_received_) pose_received_ = true;
}

bool MPPI::findTargetPoint()
{
  // Ensure closest_idx_ is within bounds
  if (static_cast<size_t>(closest_idx_) >= current_ref_path_.poses.size()) {
    closest_idx_ = 0;
  }
  
  double min_dist = std::numeric_limits<double>::max();
  
  // Find closest point, avoid going out of bounds
  size_t max_idx = current_ref_path_.poses.size() > 1 ? current_ref_path_.poses.size() - 1 : 1;
  for (size_t i = static_cast<size_t>(closest_idx_); i < max_idx; ++i) {
    double dist = calc_dinstanse(i);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx_ = i;
    }
  }

  //　一旦lookahead_distance_は使わず近い点をターゲットにする
  target_point_.resize(3);
  target_point_[0] = current_ref_path_.poses[closest_idx_].pose.position.x;
  target_point_[1] = current_ref_path_.poses[closest_idx_].pose.position.y;
  target_point_[2] = tf2::getYaw(current_ref_path_.poses[closest_idx_].pose.orientation);
  
  // //ルックアヘッド距離に基づいてターゲットポイントを決定
  // double min_proj_dist = std::numeric_limits<double>::max();
  // target_idx_ = closest_idx_;
  // for (size_t i = closest_idx_; i < current_path_->poses.size(); ++i) {
  //   double dist = std::hypot(
  //     current_path_->poses[i].pose.position.x - current_pose_.position.x,
  //     current_path_->poses[i].pose.position.y - current_pose_.position.y);
  //   double proj_dist =  abs(dist - lookahead_distance_);
  //   if (proj_dist < min_proj_dist) {
  //     min_proj_dist = proj_dist;
  //     target_idx_ = i;
  //   }
  // }  
  // target_point_ = current_path_->poses[target_idx_].pose.position;
  return true;
}

double MPPI::calc_dinstanse(size_t target_idx)
{
  // Safety check for bounds
  if (target_idx >= current_ref_path_.poses.size() - 1) {
    // Return distance to the last point
    const auto& p = current_ref_path_.poses.back().pose.position;
    return std::hypot(current_pose_[0] - p.x, current_pose_[1] - p.y);
  }
  
  const auto& p1 = current_ref_path_.poses[target_idx].pose.position;
  const auto& p2 = current_ref_path_.poses[target_idx + 1].pose.position;
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double seg_len2 = dx * dx + dy * dy;
  if (seg_len2 < 1e-6) {
    return std::hypot(current_pose_[0] - p1.x, current_pose_[1] - p1.y); // 同じ点の場合は直接距離
  }
  double t = ((current_pose_[0] - p1.x) * dx +
              (current_pose_[1] - p1.y) * dy)
              / seg_len2;
  t = std::clamp(t, 0.0, 1.0);
  geometry_msgs::msg::Point proj;
  proj.x = p1.x + t * dx;
  proj.y = p1.y + t * dy;
  proj.z = p1.z + t * (p2.z - p1.z);
  double dist = std::hypot(
    current_pose_[0] - proj.x,
    current_pose_[1] - proj.y);
  return dist;
}

void MPPI::CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    costmap_received_ = true;
    costmap_ = *msg;
    mppi_controller_->setCostMap(costmap_);
}

bool MPPI::targetReached()
{   
  double goal_x = current_ref_path_.poses[closest_idx_].pose.position.x;
  double goal_y = current_ref_path_.poses[closest_idx_].pose.position.y;
  // double goal_x = current_ref_path_.poses.back().pose.position.x;
  // double goal_y = current_ref_path_.poses.back().pose.position.y;
  double dist_to_goal = std::hypot(current_pose_[0] - goal_x, current_pose_[1] - goal_y);
  return dist_to_goal < goal_tolerance_;
}

void MPPI::controlLoop()
{
    // 1. Get current pose
    getCurrentPose();

    if (!costmap_received_ || !ref_path_received_  || !pose_received_) {
      geometry_msgs::msg::Twist stop_msg;
      cmd_vel_pub_->publish(stop_msg);
      return;
    }
  
    //TODO 一定期間コストマップが更新されなければ停止
    
    if (targetReached()) {
      if (closest_idx_ >= current_ref_path_.poses.size() - 1) {
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        ref_path_received_ = false;
        RCLCPP_INFO(get_logger(), "Goal reached, stopping");
      }else {
        closest_idx_++;
      }
      return;
    }

    // 2. Find target point on the reference path
    if (!findTargetPoint()) {
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        RCLCPP_INFO(get_logger(), "Failed to find target point");
        return;
    }
    
    // Compute control command using MPPI
    VelocityCommand control_command = mppi_controller_->solveMPPI(
      current_pose_, 
      target_point_
    );

    // Publish control command
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = control_command[0];
    cmd_vel.angular.z = control_command[1];
    cmd_vel_pub_->publish(cmd_vel);

    // // Visualization
    // if (enable_visualization_) {
    //     publishOptimalTrajectory();
    //     if (enable_sampled_paths_visualization_) {
    //         publishSampledTrajectories();
    //     }
    // }
}
} // namespace mppi

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mppi::MPPI)

