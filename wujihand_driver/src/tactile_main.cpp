#include "rclcpp/rclcpp.hpp"
#include "wujihand_driver/tactile_driver_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<wujihand_driver::TactileDriverNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("tactile_driver"),
                     "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
