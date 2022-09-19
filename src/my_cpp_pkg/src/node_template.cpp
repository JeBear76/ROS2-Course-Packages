#include "rclcpp/rclcpp.hpp"

class MyCustomNode : public rclcpp::Node
{
public:
    MyCustomNode(std::string nodeName) : Node(nodeName)
    {
        _nodeName = nodeName;
        RCLCPP_INFO(this->get_logger(), (std::string("ROS2 ") + nodeName + std::string(" C++ started.")).c_str());
    }

private:
    std::string _nodeName;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(std::string("node_name"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}