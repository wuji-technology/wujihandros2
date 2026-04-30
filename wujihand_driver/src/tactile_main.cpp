#include "rclcpp/rclcpp.hpp"
#include "wujihand_driver/tactile_driver_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<wujihand_driver::TactileDriverNode>();
        // Multi-threaded executor so the 100 ms diagnostics timer (which
        // issues a 2-second-timeout SDK command) cannot starve service
        // callbacks. The node assigns the timer and the three services to
        // distinct mutually-exclusive callback groups.
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("tactile_driver"),
                     "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
