#ifndef DIFF_DRIVE_HPP
#define DIFF_DRIVE_HPP

#include <cmath>
#include <Eigen/Dense>
#include <mppi_planner/common.hpp>

class DiffDrive
{
public:
    DiffDrive(double dt)
    {
        dt_ = dt;
    }
    ~DiffDrive() {}

    std::vector<double> update_state(std::vector<double> x, const std::vector<double>& u) {
        kinematics(x, u);
        return x;
    }

private:
    //state 
    double dt_;

    void kinematics(std::vector<double>& x, const std::vector<double>& u) {
        double v = u[0];
        double w = u[1];

        x[0] += std::cos(x[2]) * v * dt_;
        x[1] += std::sin(x[2]) * v * dt_;
        x[2] += w * dt_;
        normalizeAngle(x[2]);
    }

    void normalizeAngle(double& angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
    }
};

#endif

