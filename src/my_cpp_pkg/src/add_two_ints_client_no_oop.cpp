#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using example_interfaces::srv::AddTwoInts;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(std::string("add_two_ints_client_no_oop"));
    auto client = node->create_client<AddTwoInts>("additionizer");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(node->get_logger(), "Waiting for service...");
    }
    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = 12;
    request->b = 67;

    auto future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        RCLCPP_INFO(node->get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Error calling service");
    }

    rclcpp::shutdown();
    return 0;
}