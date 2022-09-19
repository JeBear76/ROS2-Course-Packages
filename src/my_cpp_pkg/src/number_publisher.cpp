#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using example_interfaces::msg::Int64;

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode(std::string nodeName) : Node(nodeName)
    {
        _nodeName = nodeName;

        _publisher = this->create_publisher<Int64>("pub593", 10);
        _timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&NumberPublisherNode::publishNumber, this));

        RCLCPP_INFO(this->get_logger(), (std::string("ROS2 ") + nodeName + std::string(" C++ started.")).c_str());
    }

private:
    void publishNumber()
    {
        Int64 msg = Int64();
        msg.data = 593;
        _publisher->publish(msg);
    }
    std::string _nodeName;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<Int64>::SharedPtr _publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>(std::string("number_publisher"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}