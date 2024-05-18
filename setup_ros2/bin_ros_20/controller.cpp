#include <iostream>
#include <string>
#include <regex>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

std::string pub_sub_topic = "topic";
std::chrono::duration<int, std::milli> pub_interval = 50ms;

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        std::cout << "Publishing" << std::endl;
        std::cout << "Command format: <step>,<angle>" << std::endl;
        std::cout << "Example: 1,90" << std::endl;
        publisher_ = this->create_publisher<std_msgs::msg::String>(pub_sub_topic, 10);
        timer_ = this->create_wall_timer(
            pub_interval, std::bind(&CommandPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        std::getline(std::cin, message.data);
        valid_input = std::regex_match(message.data, command_regex);
        if (valid_input) {
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        } else {
            RCLCPP_INFO(this->get_logger(), "Invalid input: '%s'", message.data.c_str());
        }
        if (count_++ > 1) {
            rclcpp::shutdown();
        }
        rclcpp::shutdown();
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    bool valid_input = false;
    // infinit step, 0 to 360 degrees
    const std::regex command_regex = std::regex("([0-9]+),(\\b([1-2]?[0-9]?[0-9]|3[0-5][0-9]|360)\\b)");
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
