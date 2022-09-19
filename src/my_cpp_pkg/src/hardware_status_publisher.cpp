#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

using my_robot_interfaces::msg::HardwareStatus;

class HardwareStatusPublisherNode : public rclcpp::Node
{
public:
    HardwareStatusPublisherNode(std::string nodeName) : Node(nodeName)
    {
        _nodeName = nodeName;
        _publisher = this->create_publisher<HardwareStatus>("robot_status", 10);
        _timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&HardwareStatusPublisherNode::publishHardwareStatus, this));
        RCLCPP_INFO(this->get_logger(), (std::string("ROS2 ") + nodeName + std::string(" C++ started.")).c_str());
    }

private:
    void publishHardwareStatus()
    {
        auto msg = HardwareStatus();
        msg.temperature = 22;
        msg.are_motors_ready = true;
        msg.debug_message = "Good to go!!!!";
        _publisher->publish(msg);
    }
    std::string _nodeName;
    rclcpp::Publisher<HardwareStatus>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>(std::string("node_name"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}