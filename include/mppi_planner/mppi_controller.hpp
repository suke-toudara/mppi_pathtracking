#ifndef MPPI_CONTROLLER_HPP
#define MPPI_CONTROLLER_HPP

#include "mppi_planner/common.hpp"
#include "mppi_planner/diff_drive.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <random>
#include <omp.h> 

namespace mppi_core
{


class MPPICore
{
public:
    MPPICore(const MPPIParams& params);
    ~MPPICore();
    
    VelocityCommand solveMPPI(
        const Pose2D& current_state,
        const Pose2D& target_point
    );
    
    void setCostMap(const nav_msgs::msg::OccupancyGrid& costmap);
    
    double getStateCost();
    double getControlCost();

private:
    void setParams(const MPPIParams& params);
    NoiseSamples generateNoise();
    void computeWeights();
    double stage_cost(const std::vector<double>& X, const std::vector<double>& U, const Pose2D& target_point, const std::vector<double>& U_prev);
    double terminal_cost(const std::vector<double>& X, const Pose2D& target_point);

    // MPPI parameters
    int K_;  // Number of samples
    int T_;  // Time horizon
    double dt_;  // Time step
    double lambda_;  // Temperature parameter
    double alpha_;  // Noise decay rate
    double exploration_rate_;  // Exploration rate
    std::vector<double> sigma_;  // Noise standard deviation
    double max_linear_velocity_;
    double max_angular_velocity_;

    // State and control variables
    StateSeqSamples state_sample_;
    ControlSeqSamples u_samples_;
    ControlSeq u_opt_seq_;
    ControlSeq u_opt_seq_latest_;
    StateSeq x_opt_seq_;
    std::vector<double> costs_;
    std::vector<double> weights_;
    NoiseSamples noises_;

    // Robot model
    std::shared_ptr<DiffDrive> model_;

    // Costmap variables
    nav_msgs::msg::OccupancyGrid local_costmap_;
    double local_costmap_origin_x_;
    double local_costmap_origin_y_;
    double local_costmap_resolution_;
    int map_size_;
    std::vector<int8_t> obstacle_grid_;

    // Target point
    Pose2D target_point_;

    // Cost tracking
    double state_cost_;
    double control_cost_;
    
    // Previous control input for acceleration cost
    Control u_prev_;
    
    // Random number generation
    static const int random_seed_ = 623;
    std::mt19937 generator_;
    
    // Timing
    double calc_time_;
};
} // namespace mppi_core

#endif