#include "drone_state/drone_state.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DroneState>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
