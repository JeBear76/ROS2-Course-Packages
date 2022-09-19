#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/battery_status.hpp"

using my_robot_interfaces::msg::BatteryStatus;

class BatteryLevelSetter : public rclcpp::Node
{
public:
    BatteryLevelSetter(std::string nodeName) : Node(nodeName)
    {
        _nodeName = nodeName;
        this->declare_parameter("required_level", 75);

        auto publisher = this->create_publisher<BatteryStatus>("battery_level", 10);
        auto batteryLevel = BatteryStatus();
        batteryLevel.battery_level = this->get_parameter("required_level").get_value<int32_t>();
        publisher->publish(batteryLevel);
        RCLCPP_INFO(this->get_logger(), (std::string("ROS2 ") + nodeName + std::string(" C++ started.")).c_str());
    }

private:
    std::string _nodeName;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryLevelSetter>(std::string("battery_level_setter"));
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}