#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using example_interfaces::srv::AddTwoInts;

class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient(std::string nodeName) : Node(nodeName)
    {
        _nodeName = nodeName;
        RCLCPP_INFO(this->get_logger(), (std::string("ROS2 ") + nodeName + std::string(" C++ started.")).c_str());
        _workers.push_back(std::thread(std::bind(&AddTwoIntsClient::callAddToIntService, this, 32, 67)));
        _workers.push_back(std::thread(std::bind(&AddTwoIntsClient::callAddToIntService, this, 26, 93)));
        _workers.push_back(std::thread(std::bind(&AddTwoIntsClient::callAddToIntService, this, 42, 67)));
        _workers.push_back(std::thread(std::bind(&AddTwoIntsClient::callAddToIntService, this, 3, 55)));
        _workers.push_back(std::thread(std::bind(&AddTwoIntsClient::callAddToIntService, this, 326, 7)));
    }

private:
    std::string _nodeName;
    std::vector<std::thread> _workers;

    void callAddToIntService(int a, int b)
    {
        auto client = this->create_client<AddTwoInts>("additionizer");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }
        auto request = std::make_shared<AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error calling service");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClient>(std::string("add_two_ints_client"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}