#ifndef COMMON_HPP
#define COMMON_HPP

#include <iostream>
#include <array>
#include <vector>
#include <algorithm>

// Type definitions for MPPI - all using std::vector for simplicity
using Pose2D = std::vector<double>;      // Pose vector [x, y, yaw]
using State = std::vector<double>;       // State vector [x, y, yaw]
using Control = std::vector<double>;     // Control vector [linear_vel, angular_vel]
using StateSeq = std::vector<State>;     // State sequence over time
using ControlSeq = std::vector<Control>; // Control sequence over time
using StateSeqSamples = std::vector<StateSeq>;   // Multiple state sequences (samples)
using ControlSeqSamples = std::vector<ControlSeq>; // Multiple control sequences (samples)
using NoiseSeq = std::vector<Control>;   // Noise sequence for one sample (same structure as Control)
using NoiseSamples = std::vector<NoiseSeq>; // Noise for all samples
using VelocityCommand = std::vector<double>; // [linear_velocity, angular_velocity]

struct MPPIParams
{
    int K; // Number of samples
    int T; // Time horizon
    int UDIM; // Control input dimension
    int XDIM; // State dimension
    double dt; // Time step
    double lambda; // Temperature parameter
    double alpha; // Noise decay rate
    double exploration_rate; // Exploration rate
    std::vector<double> sigma; // Noise standard deviation
    double max_linear_velocity;
    double max_angular_velocity;

    // weight
    double weight_goal;
    double weight_obstacle;
    double weight_velocity;
};

#endif // COMMON_HPP