#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using example_interfaces::msg::Int64;
using example_interfaces::srv::SetBool;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode(std::string nodeName) : Node(nodeName), _counter(0)
    {
        _nodeName = nodeName;
        _subscription = this->create_subscription<Int64>("pub593", 10, std::bind(&NumberCounterNode::numberReceived, this, std::placeholders::_1));
        _publisher = this->create_publisher<Int64>("countifier", 10);
        _resetCounterService = this->create_service<SetBool>("reset_counter", std::bind(&NumberCounterNode::resetCounter, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), (std::string("ROS2 ") + nodeName + std::string(" C++ started.")).c_str());
    }

private:
    void resetCounter(const SetBool::Request::SharedPtr request, const SetBool::Response::SharedPtr response)
    {
        if (request->data)
        {
            _counter = 0;
            response->success = true;
            response->message = "Reset!";
            return;
        }
        response->success = false;
        response->message = "Did nothing, as requested.";
    }

    void numberReceived(Int64 msg)
    {
        RCLCPP_INFO(this->get_logger(), (std::string("ROS2 ") + _nodeName + std::string(" received ") + std::to_string(msg.data) + std::string(" ") + std::to_string(++_counter) + std::string(" times")).c_str());
        Int64 cnt = Int64();
        cnt.data = _counter;
        _publisher->publish(cnt);
    }

    std::string _nodeName;
    long _counter;
    rclcpp::Subscription<Int64>::SharedPtr _subscription;
    rclcpp::Publisher<Int64>::SharedPtr _publisher;
    rclcpp::Service<SetBool>::SharedPtr _resetCounterService;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>(std::string("number_counter"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}