#include "mppi_planner/mppi_component.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto component = std::make_shared<mppi::MPPI>(options);
    exec.add_node(component);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
