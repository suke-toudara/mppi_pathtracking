#include "mppi_planner/mppi_component.hpp"
#include <cmath>
#include <limits>

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
    mppi_sample_traj_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("sampled_trajectories", 10);
    // subscriber
    ref_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "reference_path", rclcpp::QoS(10).reliable().durability_volatile().keep_last(1),
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
    start_turn_in_place_ = declare_parameter<bool>("start_turn_in_place", true);
    start_turn_angle_threshold_ = declare_parameter<double>("start_turn_angle_threshold", 1.0);
    start_turn_release_threshold_ = declare_parameter<double>("start_turn_release_threshold", 0.6);
    start_turn_angular_speed_ = declare_parameter<double>("start_turn_angular_speed", 0.6);
    start_turn_max_distance_ = declare_parameter<double>("start_turn_max_distance", 1.0);
    loop_detection_tolerance_ = declare_parameter<double>("loop_detection_tolerance", 0.2);
    loop_lookahead_distance_ = declare_parameter<double>("loop_lookahead_distance", 1.0);
    loop_exit_distance_ = declare_parameter<double>("loop_exit_distance", 1.0);
    
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
    mppi_params_.min_linear_velocity = declare_parameter<double>("min_linear_velocity", 0.0);
    mppi_params_.max_linear_accel = declare_parameter<double>("max_linear_accel", 1.0);
    mppi_params_.max_angular_accel = declare_parameter<double>("max_angular_accel", 1.0);
    mppi_params_.path_distance_weight = declare_parameter<double>("path_distance_weight", 5.0);
    mppi_params_.goal_distance_weight = declare_parameter<double>("goal_distance_weight", 5.0);
    mppi_params_.path_heading_weight = declare_parameter<double>("path_heading_weight", 2.0);
    
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
  state_ = ControllerState::WAIT;

  ref_path_is_loop_ = false;
  if (msg->poses.size() > 2) {
    const auto& first = msg->poses.front().pose.position;
    const auto& last = msg->poses.back().pose.position;
    const double loop_dist = std::hypot(first.x - last.x, first.y - last.y);
    ref_path_is_loop_ = loop_dist <= loop_detection_tolerance_;
  }
  if (!msg->poses.empty()) {
    loop_start_x_ = msg->poses.front().pose.position.x;
    loop_start_y_ = msg->poses.front().pose.position.y;
  }
  loop_has_left_start_ = false;

  Path2D path_points;
  path_points.reserve(msg->poses.size());
  for (const auto& pose : msg->poses) {
    path_points.push_back({pose.pose.position.x, pose.pose.position.y});
  }
  mppi_controller_->setReferencePath(path_points);
  
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
  const size_t path_size = current_ref_path_.poses.size();
  if (path_size == 0) {
    return false;
  }

  // Update closest index for diagnostics
  if (static_cast<size_t>(closest_idx_) >= path_size) {
    closest_idx_ = 0;
  }
  double min_dist = std::numeric_limits<double>::max();
  size_t max_idx = path_size > 1 ? path_size - 1 : 1;
  for (size_t i = static_cast<size_t>(closest_idx_); i < max_idx; ++i) {
    double dist = calc_dinstanse(i);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx_ = i;
    }
  }

  if (ref_path_is_loop_ && loop_lookahead_distance_ > 0.0) {
    size_t effective_size = path_size;
    if (path_size > 1) {
      const auto& first = current_ref_path_.poses.front().pose.position;
      const auto& last = current_ref_path_.poses.back().pose.position;
      const double loop_dist = std::hypot(first.x - last.x, first.y - last.y);
      if (loop_dist <= loop_detection_tolerance_) {
        effective_size = path_size - 1;
      }
    }
    if (effective_size < 2) {
      return false;
    }
    size_t idx = static_cast<size_t>(closest_idx_) % effective_size;
    double accum = 0.0;
    size_t steps = 0;
    while (steps < effective_size && accum < loop_lookahead_distance_) {
      const size_t next = (idx + 1) % effective_size;
      const auto& p0 = current_ref_path_.poses[idx].pose.position;
      const auto& p1 = current_ref_path_.poses[next].pose.position;
      accum += std::hypot(p1.x - p0.x, p1.y - p0.y);
      idx = next;
      ++steps;
    }
    const auto& target_pose = current_ref_path_.poses[idx].pose;
    target_point_.resize(3);
    target_point_[0] = target_pose.position.x;
    target_point_[1] = target_pose.position.y;
    target_point_[2] = tf2::getYaw(target_pose.orientation);
  } else {
    // Use path end as goal; path distance cost handles path tracking.
    const auto& goal_pose = current_ref_path_.poses.back().pose;
    target_point_.resize(3);
    target_point_[0] = goal_pose.position.x;
    target_point_[1] = goal_pose.position.y;
    target_point_[2] = tf2::getYaw(goal_pose.orientation);
  }
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
  // シンプルなゴール判定:
  // - 非ループパス: ゴール点からの距離が goal_tolerance 以内
  // - ループパス: スタート地点から離れた後、再びスタート地点に戻った
  
  if (current_ref_path_.poses.empty()) {
    return false;
  }
  
  // ゴール点（パスの最終点）
  const double goal_x = current_ref_path_.poses.back().pose.position.x;
  const double goal_y = current_ref_path_.poses.back().pose.position.y;
  const double dist_to_goal = std::hypot(current_pose_[0] - goal_x, current_pose_[1] - goal_y);
  
  if (ref_path_is_loop_) {
    // ループパスの場合: スタート地点から離れた後、戻ってきたらゴール
    if (!loop_has_left_start_) {
      // まだスタート地点から離れていない
      return false;
    }
    // スタート地点に戻ってきたか
    const double dist_to_start = std::hypot(
      current_pose_[0] - loop_start_x_, current_pose_[1] - loop_start_y_);
    
    RCLCPP_DEBUG(get_logger(), "Loop path: dist_to_start=%.3f, goal_tolerance=%.3f, left_start=%d",
      dist_to_start, goal_tolerance_, loop_has_left_start_);
    
    return dist_to_start <= goal_tolerance_;
  } else {
    // 非ループパスの場合: ゴール点に到達したらゴール
    RCLCPP_DEBUG(get_logger(), "Non-loop path: dist_to_goal=%.3f, goal_tolerance=%.3f",
      dist_to_goal, goal_tolerance_);
    
    return dist_to_goal <= goal_tolerance_;
  }
}

void MPPI::controlLoop()
{
    // 1. Get current pose
    getCurrentPose();

    if (!costmap_received_ || !ref_path_received_  || !pose_received_) {
      geometry_msgs::msg::Twist stop_msg;
      cmd_vel_pub_->publish(stop_msg);
      state_ = ControllerState::WAIT;
      return;
    }
  
    //TODO 一定期間コストマップが更新されなければ停止
    
    if (ref_path_is_loop_ && !loop_has_left_start_) {
      const double dist_to_start = std::hypot(
        current_pose_[0] - loop_start_x_, current_pose_[1] - loop_start_y_);
      if (dist_to_start > loop_exit_distance_) {
        loop_has_left_start_ = true;
        RCLCPP_INFO(get_logger(), "Loop: Left start area (dist=%.3f > %.3f)", 
          dist_to_start, loop_exit_distance_);
      }
    }

    switch (state_) {
      case ControllerState::WAIT: {
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        clearSampledTrajectories();

        if (!start_turn_in_place_) {
          state_ = ControllerState::TRACK;
          return;
        }
        if (!current_ref_path_.poses.empty()) {
          const auto& start_pos = current_ref_path_.poses.front().pose.position;
          const double dist_to_start = std::hypot(
            current_pose_[0] - start_pos.x, current_pose_[1] - start_pos.y);
          if (dist_to_start > start_turn_max_distance_) {
            state_ = ControllerState::TRACK;
            return;
          }
          double path_heading = 0.0;
          if (computeStartPathHeading(path_heading)) {
            const double yaw_error = normalizeAngle(path_heading - current_pose_[2]);
            if (std::abs(yaw_error) > start_turn_angle_threshold_) {
              state_ = ControllerState::ROTATE;
              return;
            }
          }
        }
        state_ = ControllerState::TRACK;
        return;
      }
      case ControllerState::ROTATE: {
        geometry_msgs::msg::Twist turn_msg;
        if (!start_turn_in_place_) {
          state_ = ControllerState::TRACK;
          return;
        }
        double path_heading = 0.0;
        if (!computeStartPathHeading(path_heading)) {
          state_ = ControllerState::WAIT;
          return;
        }
        const double yaw_error = normalizeAngle(path_heading - current_pose_[2]);
        if (std::abs(yaw_error) <= start_turn_release_threshold_) {
          state_ = ControllerState::TRACK;
          return;
        }
        turn_msg.angular.z = (yaw_error >= 0.0) ? start_turn_angular_speed_ : -start_turn_angular_speed_;
        cmd_vel_pub_->publish(turn_msg);
        return;
      }
      case ControllerState::TRACK:
      default:
        break;
    }

    // ゴール判定
    const double goal_x = current_ref_path_.poses.back().pose.position.x;
    const double goal_y = current_ref_path_.poses.back().pose.position.y;
    const double dist_to_goal = std::hypot(current_pose_[0] - goal_x, current_pose_[1] - goal_y);
    
    // 定期的に距離をログ出力（デバッグ用）
    static int log_counter = 0;
    if (++log_counter >= 20) {  // 約1秒ごと（20Hz想定）
      log_counter = 0;
      if (ref_path_is_loop_) {
        const double dist_to_start = std::hypot(
          current_pose_[0] - loop_start_x_, current_pose_[1] - loop_start_y_);
        RCLCPP_INFO(get_logger(), "Loop path: dist_to_start=%.3f, left_start=%s, tolerance=%.3f",
          dist_to_start, loop_has_left_start_ ? "true" : "false", goal_tolerance_);
      } else {
        RCLCPP_INFO(get_logger(), "Path tracking: dist_to_goal=%.3f, tolerance=%.3f",
          dist_to_goal, goal_tolerance_);
      }
    }
    
    if (targetReached()) {
      geometry_msgs::msg::Twist stop_msg;
      cmd_vel_pub_->publish(stop_msg);
      ref_path_received_ = false;
      clearSampledTrajectories();
      state_ = ControllerState::WAIT;
      RCLCPP_INFO(get_logger(), "=== GOAL REACHED! Stopping. ===");
      return;
    }

    // 2. Find target point on the reference path
    if (!findTargetPoint()) {
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        RCLCPP_INFO(get_logger(), "Failed to find target point");
        state_ = ControllerState::WAIT;
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

    if (enable_sampled_paths_visualization_) {
      publishSampledTrajectories();
    }
}

void MPPI::publishSampledTrajectories()
{
  if (!mppi_sample_traj_markers_pub_) {
    return;
  }

  const auto& samples = mppi_controller_->getStateSamples();
  if (samples.empty()) {
    return;
  }

  const int total_samples = static_cast<int>(samples.size());
  const int max_display = std::max(1, max_sampled_paths_display_);
  const int display_count = std::min(max_display, total_samples);

  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.stamp = now();
  clear_marker.header.frame_id = global_frame_id_;
  clear_marker.ns = "mppi_sample_paths";
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  int marker_id = 0;
  for (int n = 0; n < display_count; ++n) {
    const int index = (display_count == 1)
      ? 0
      : (n * (total_samples - 1)) / (display_count - 1);
    const auto& sample = samples[index];
    if (sample.empty()) {
      continue;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = global_frame_id_;
    marker.ns = "mppi_sample_paths";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = 1.0f;
    marker.color.g = 0.55f;
    marker.color.b = 0.0f;
    marker.color.a = 0.9f;

    marker.points.reserve(sample.size());
    for (const auto& state : sample) {
      if (state.size() < 2) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = state[0];
      p.y = state[1];
      p.z = 0.0;
      marker.points.push_back(p);
    }

    marker_array.markers.push_back(marker);
  }

  mppi_sample_traj_markers_pub_->publish(marker_array);
}

void MPPI::clearSampledTrajectories()
{
  if (!mppi_sample_traj_markers_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.stamp = now();
  clear_marker.header.frame_id = global_frame_id_;
  clear_marker.ns = "mppi_sample_paths";
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  mppi_sample_traj_markers_pub_->publish(marker_array);
}

bool MPPI::computePathHeading(double& heading_yaw) const
{
  if (current_ref_path_.poses.size() < 2) {
    return false;
  }
  size_t path_size = current_ref_path_.poses.size();
  if (ref_path_is_loop_ && path_size > 1) {
    const auto& first = current_ref_path_.poses.front().pose.position;
    const auto& last = current_ref_path_.poses.back().pose.position;
    const double loop_dist = std::hypot(first.x - last.x, first.y - last.y);
    if (loop_dist <= loop_detection_tolerance_) {
      path_size -= 1;
    }
  }
  if (path_size < 2) {
    return false;
  }
  size_t idx = static_cast<size_t>(closest_idx_) % path_size;
  const size_t next = (idx + 1) % path_size;
  const auto& p0 = current_ref_path_.poses[idx].pose.position;
  const auto& p1 = current_ref_path_.poses[next].pose.position;
  heading_yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
  return true;
}

bool MPPI::computeStartPathHeading(double& heading_yaw) const
{
  if (current_ref_path_.poses.size() < 2) {
    return false;
  }
  const auto& first = current_ref_path_.poses.front().pose.position;
  for (size_t i = 1; i < current_ref_path_.poses.size(); ++i) {
    const auto& p = current_ref_path_.poses[i].pose.position;
    const double dx = p.x - first.x;
    const double dy = p.y - first.y;
    if (std::hypot(dx, dy) > 1e-6) {
      heading_yaw = std::atan2(dy, dx);
      return true;
    }
  }
  return false;
}

double MPPI::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

} // namespace mppi

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mppi::MPPI)

