#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using example_interfaces::srv::AddTwoInts;
using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServer : public rclcpp::Node
{
public:
    AddTwoIntsServer(std::string nodeName) : Node(nodeName)
    {
        _nodeName = nodeName;
        _service = this->create_service<AddTwoInts>("additionizer", std::bind(&AddTwoIntsServer::additionizerCallback, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), (std::string("ROS2 ") + nodeName + std::string(" C++ started.")).c_str());
    }

private:
    void additionizerCallback(const AddTwoInts::Request::SharedPtr request,
                              const AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
    }

    std::string _nodeName;
    rclcpp::Service<AddTwoInts>::SharedPtr _service;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServer>(std::string("add_two_ints_server"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}