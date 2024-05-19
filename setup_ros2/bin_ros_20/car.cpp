#include <iostream>
#include <string>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

std::string pub_sub_topic = "controll_signal";
std::chrono::duration<int, std::milli> pub_interval = 100ms;
std::chrono::duration<int, std::milli> sub_interval = 10s;
std::chrono::duration<int, std::milli> sleep_interval = 2s;
long unsigned int consecutive_publish_cnt = 10;

class ReadyPublisher : public rclcpp::Node
{
public:
    ReadyPublisher()
    : Node("ready_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(pub_sub_topic, 10);
        timer_ = this->create_wall_timer(
            pub_interval, std::bind(&ReadyPublisher::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Publishing: 'ready'");
    }

private:
    void timer_callback()
    {
        ++count_;
        auto message = std_msgs::msg::String();
        message.data = "ready";
        publisher_->publish(message);
        if (count_ >= consecutive_publish_cnt) {
            rclcpp::shutdown();
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

class TermPublisher : public rclcpp::Node
{
public:
    TermPublisher()
    : Node("terminate_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(pub_sub_topic, 10);
        timer_ = this->create_wall_timer(
            pub_interval, std::bind(&TermPublisher::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Publishing: 'ready'");
    }

private:
    void timer_callback()
    {
        ++count_;
        auto message = std_msgs::msg::String();
        message.data = "terminate";
        publisher_->publish(message);
        if (count_ >= consecutive_publish_cnt) {
            rclcpp::shutdown();
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    bool ready = false;
};

class CommandReceiver : public rclcpp::Node
{
public:
    CommandReceiver()
    : Node("command_receiver")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        pub_sub_topic, 10, std::bind(&CommandReceiver::topic_callback, this, _1));

        timer_ = this->create_wall_timer(
            sub_interval, std::bind(&CommandReceiver::timer_callback, this));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    void timer_callback()
    {
        //RCLCPP_INFO(this->get_logger(), "Subscribe timeout ...");
        rclcpp::shutdown();
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

void publish_ready_once(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadyPublisher>());
    rclcpp::shutdown();
}

void publish_term_once(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TermPublisher>());
    rclcpp::shutdown();
}

void subscribe_once(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandReceiver>());
    rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
    std::cout << "Program starting ..." << std::endl;

    while (true) {
        std::cout << "Sending ready signal ..." << std::endl;
        publish_ready_once(argc, argv);
        std::cout << "Waiting for command ..." << std::endl;
        subscribe_once(argc, argv);
        publish_term_once(argc, argv);
        std::cout << "Sending terminate signal ..." << std::endl;
        std::this_thread::sleep_for(sleep_interval);
    }

    return 0;
}
