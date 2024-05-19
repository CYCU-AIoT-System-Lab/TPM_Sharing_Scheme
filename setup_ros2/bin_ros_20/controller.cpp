#include <iostream>
#include <string>
#include <regex>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

std::string pub_sub_topic = "controll_signal";
std::chrono::duration<int, std::milli> pub_interval = 10ms;
std::chrono::duration<int, std::milli> sub_interval = 1s;
long unsigned int consecutive_publish_cnt = 1;
bool ready = false;

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher()
    : Node("command_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(pub_sub_topic, 10);
        timer_ = this->create_wall_timer(
            pub_interval, std::bind(&CommandPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        std::cout << "Enter command: ";
        std::getline(std::cin, message.data);
        valid_input = std::regex_match(message.data, command_regex);
        if (valid_input) {
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
            ++count_;
        } else {
            RCLCPP_INFO(this->get_logger(), "Invalid input: '%s'", message.data.c_str());
        }
        if (count_ >= consecutive_publish_cnt) {
            rclcpp::shutdown();
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    bool valid_input = false;
    // infinit step, 0 to 360 degrees
    const std::regex command_regex = std::regex("([0-9]+),(\\b([1-2]?[0-9]?[0-9]|3[0-5][0-9]|360)\\b)");
};

class ReadyCommandReceiver : public rclcpp::Node
{
public:
    ReadyCommandReceiver()
    : Node("ready_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        pub_sub_topic, 10, std::bind(&ReadyCommandReceiver::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        if (msg->data == "ready") {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            ready = true;
            rclcpp::shutdown();
        }
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class TermCommandReceiver : public rclcpp::Node
{
public:
    TermCommandReceiver()
    : Node("terminate_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        pub_sub_topic, 10, std::bind(&TermCommandReceiver::topic_callback, this, _1));

        timer_ = this->create_wall_timer(
            sub_interval, std::bind(&TermCommandReceiver::timer_callback, this));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        if (msg->data == "terminate") {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            ready = false;
            rclcpp::shutdown();
        }
    }
    void timer_callback()
    {
        //RCLCPP_INFO(this->get_logger(), "Subscribe timeout ...");
        rclcpp::shutdown();
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

void publish_once(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPublisher>());
    rclcpp::shutdown();
}

void subscribe_ready(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadyCommandReceiver>());
    rclcpp::shutdown();
}

void subscribe_term(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TermCommandReceiver>());
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    std::cout << "Program starting ..." << std::endl;
    std::cout << "Command format: <step>,<angle>" << std::endl;
    std::cout << "Example: 1,90" << std::endl;

    while (true) {
        std::cout << "Waiting for ready signal ..." << std::endl;
        subscribe_ready(argc, argv); // wait for ready signal
        std::cout << "Ready signal received, start publishing until terminiate signal ..." << std::endl;
        while (ready) {
            publish_once(argc, argv);
            subscribe_term(argc, argv); // wait for term signal
        }
    }
    return 0;
}
