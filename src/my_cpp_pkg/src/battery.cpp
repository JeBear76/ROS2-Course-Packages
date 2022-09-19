#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/battery_status.hpp"

using my_robot_interfaces::msg::BatteryStatus;
using my_robot_interfaces::srv::SetLED;
using std::placeholders::_1;
using std::placeholders::_2;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode(std::string nodeName) : Node(nodeName)
    {
        this->declare_parameter<int>("battery_low_led_position", 2);
        _nodeName = nodeName;
        _battery_low_led_position = this->get_parameter("battery_low_led_position").get_value<int32_t>();
        _setLedClient = this->create_client<SetLED>("led_panel_update");
        _batteryListener = this->create_subscription<BatteryStatus>("battery_level", 10, std::bind(&BatteryNode::batteryStatusReceived, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), (std::string("ROS2 ") + nodeName + std::string(" C++ started.")).c_str());
    }

private:
    void batteryStatusReceived(const BatteryStatus status)
    {
        RCLCPP_INFO(this->get_logger(), (std::string("Received ") + std::to_string(status.battery_level) + " battery power...").c_str());
        bool state = 0;
        if (status.battery_level < 50)
        {
            state = 1;
        }
        else
        {
            state = 0;
        }

        _setLedClient->prune_pending_requests();

        _workers.push_back(std::thread(std::bind(&BatteryNode::callSetLedService, this, state)));
    }

    void callSetLedService(const bool state)
    {
        while (!_setLedClient->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }

        auto request = std::make_shared<SetLED::Request>();

        request->position = _battery_low_led_position;
        request->state = state;

        auto fnr = _setLedClient->async_send_request(request, [this, request](const rclcpp::Client<SetLED>::SharedFuture future)
                                                     { try
                                                        {
                                                            auto response = future.get();
                                                            RCLCPP_INFO(this->get_logger(), (response->debug_response).c_str());
                                                        }
                                                        catch (const std::exception &e)
                                                        {
                                                            RCLCPP_ERROR(this->get_logger(), "Error calling service");
                                                        } });
    }

    rclcpp::Subscription<BatteryStatus>::SharedPtr _batteryListener;
    rclcpp::Client<SetLED>::SharedPtr _setLedClient;
    std::string _nodeName;
    std::vector<std::thread> _workers;
    int32_t _battery_low_led_position;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>(std::string("battery"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}