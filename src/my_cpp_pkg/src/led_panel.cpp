#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_panel_state.hpp"

using my_robot_interfaces::msg::LEDPanelState;
using my_robot_interfaces::srv::SetLED;
using std::placeholders::_1;
using std::placeholders::_2;

class LEDPanelNode : public rclcpp::Node
{
public:
    LEDPanelNode(std::string nodeName) : Node(nodeName)
    {
        _nodeName = nodeName;
        this->declare_parameter<std::vector<bool>>("startup_state", std::vector<bool>{false, false, false});
        this->ledStrip = this->get_parameter("startup_state").as_bool_array();

        _service = this->create_service<SetLED>("led_panel_update", std::bind(&LEDPanelNode::setLEDRequestReceived, this, _1, _2));
        _publisher = this->create_publisher<LEDPanelState>("led_panel_state", 10);
        RCLCPP_INFO(this->get_logger(), (std::string("ROS2 ") + nodeName + std::string(" C++ started.")).c_str());
    }

private:
    void setLEDRequestReceived(const SetLED::Request::SharedPtr request, const SetLED::Response::SharedPtr response)
    {
        int ledStripLength = ledStrip.size();
        if (request->position >= 0 && request->position < ledStripLength)
        {
            auto ledPanelState = LEDPanelState();

            ledStrip[request->position] = request->state;
            response->success = true;
            response->debug_response = "LED strip updated.";

            ledPanelState.led_statuses = ledStrip;

            _publisher->publish(ledPanelState);
            return;
        }
        response->success = false;
        response->debug_response = "No LED at this posisiton.";
    }

    rclcpp::Service<SetLED>::SharedPtr _service;
    rclcpp::Publisher<LEDPanelState>::SharedPtr _publisher;
    std::vector<bool> ledStrip;
    std::string _nodeName;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LEDPanelNode>(std::string("led_panel"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}