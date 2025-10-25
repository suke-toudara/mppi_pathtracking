
#ifndef MPPI_COMPONENT_HPP
#define MPPI_COMPONENT_HPP

#include "mppi_planner/common.hpp"
#include "mppi_planner/diff_drive.hpp"
#include "mppi_planner/mppi_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>



namespace mppi 
{
class MPPI : public rclcpp::Node
{
public:
    // constructor
    explicit MPPI(const rclcpp::NodeOptions & node_options);
    // destructor
    ~MPPI();    
private:
    // publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mppi_optimal_traj_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mppi_sample_traj_pub_;

    // subscriber
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ref_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    // timer   
    rclcpp::TimerBase::SharedPtr control_timer_;

    // MPPI parameters
    double control_frequency_;

    bool enable_visualization_;
    bool enable_optical_paths_visualization_;
    bool enable_sampled_paths_visualization_;
    int max_sampled_paths_display_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double sigma_steer_;
    double goal_tolerance_;

    // target point 
    Pose2D target_point_;

    // robot state
    std::string base_frame_id_;
    std::string global_frame_id_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    Pose2D current_pose_;
    bool pose_received_ = false;

    // costmap
    bool costmap_received_ = false;
    nav_msgs::msg::OccupancyGrid costmap_;

    // reference path
    bool ref_path_received_ = false;
    nav_msgs::msg::Path current_ref_path_;
    int closest_idx_ = 0;




    std::shared_ptr<mppi_core::MPPICore> mppi_controller_;
    std::shared_ptr<DiffDrive> robot_model_;
    
    void setParameters();
    void refPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void getCurrentPose();
    bool findTargetPoint();
    double calc_dinstanse(size_t target_idx);
    void CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void controlLoop();
    bool targetReached();
};
} // namespace mppi

#endif