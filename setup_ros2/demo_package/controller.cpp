#include <iostream>
#include <string>
#include <regex>
#include <cerrno>
#include <cstring>
#include <string>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/* custome error definition */
#define EC_CLI_NO_PI_VALUE          0x1001
#define EC_CLI_NO_SI_VALUE          0x1002
#define EC_CLI_INVALID_CPN_VALUE    0x1003
#define EC_CLI_NO_CPN_VALUE         0x1004
#define EC_CLI_INVALID_RPN_VALUE    0x1005
#define EC_CLI_NO_RPN_VALUE         0x1006
#define EC_CLI_NO_T_VALUE           0x1007
#define EC_CLI_UNKNOWN_OPTION       0x1008

/* cpp namespace */
using namespace std::chrono_literals;
using std::placeholders::_1;

/* global CLI params default values */
std::chrono::duration<int, std::milli> pub_interval = 10ms;
std::chrono::duration<int, std::milli> sub_interval = 1s;
long unsigned int consecutive_publish_num = 1;
long unsigned int repeat_publish_num = 5;
std::string pub_sub_topic = "controll_signal";
bool basic_mode = false;

/* global params */
std::string ready_sig = "ready";
std::string term_sig  = "terminate";
bool ready = false;

/*
 * @brief ROS2 Publisher node class, get user input, perform regex command matching, publish to topic
 */
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
            for (long unsigned int i=0; i<repeat_publish_num; i++) {
                publisher_->publish(message);
            }
            ++count_;
        } else {
            RCLCPP_INFO(this->get_logger(), "Invalid input: '%s'", message.data.c_str());
        }
        if (count_ >= consecutive_publish_num) {
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

/*
 * @brief ROS2 Receiver node class, wait till "ready" signal is received
 */
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
        if (msg->data == ready_sig) {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            ready = true;
            rclcpp::shutdown();
        }
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/*
 * @brief ROS2 Reciver node class, wait till "terminate" signal is received or time out itself
 */
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
        if (msg->data == term_sig) {
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

/*
 * @brief Spawn CommandPublisher node once, it will terminate itself once command published.
 */
void publish_once(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPublisher>());
    rclcpp::shutdown();
    return;
}

/*
 * @brief Spawn ReadyCommandReceiver node once, it will not terminate itself until command received.
 */
void subscribe_ready(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadyCommandReceiver>());
    rclcpp::shutdown();
    return;
}

/*
 * @brief Spawn TermCommandReceiver node once, it will terminate itself once command received or pre-set timeout reached.
 */
void subscribe_term(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TermCommandReceiver>());
    rclcpp::shutdown();
    return;
}

/*
 * @brief Print out CLI argument usage details.
 */
void print_help_message(void)
{
    std::cout << "Usage: ros2 run remote_controller controller [-pi <milli-seconds>] [-si <seconds>] [-cpn <positive_int>] [-rpn <positive_int>] [-t <topic_string>]" << std::endl;
    std::cout << "       ros2 run remote_controller controller [-b]" << std::endl;
    std::cout << "       ros2 run remote_controller controller [-h] [--help]" << std::endl;
    std::cout << "Options:\n" << std::endl;
    std::cout << "  -pi     Publish interval in milliseconds" << std::endl;
    std::cout << "  -si     Subscribe interval in seconds" << std::endl;
    std::cout << "  -cpn    Consecutive publish count, times to ask for user input and do regex match" << std::endl;
    std::cout << "  -rpn    Repeat publish count, times to publish the same message" << std::endl;
    std::cout << "  -t      ROS2 publish subscribe topic" << std::endl;
    std::cout << "  -b      Basic control mode, only send" << std::endl;
    return;
}

/*
 * @brief Print out corresponding error message when errors happens.
 */
void print_custom_errno_message(int err_no)
{
    std::string term_err_str = "\033[31mERROR: \033[0m";
    switch (err_no) {
        case EC_CLI_NO_PI_VALUE:
            printf("%s0x%x: No \"-pi\" value provided\n\n", term_err_str.c_str(), errno);
            break;
        case EC_CLI_NO_SI_VALUE:
            printf("%s0x%x: No \"-si\" value provided\n\n", term_err_str.c_str(), errno);
            break;
        case EC_CLI_INVALID_CPN_VALUE:
            printf("%s0x%x: Invalid \"-cpn\" value provided\n\n", term_err_str.c_str(), errno);
            break;
        case EC_CLI_NO_CPN_VALUE:
            printf("%s0x%x: No \"-cpn\" value provided\n\n", term_err_str.c_str(), errno);
            break;
        case EC_CLI_INVALID_RPN_VALUE:
            printf("%s0x%x: Invalid \"-rpn\" value provided\n\n", term_err_str.c_str(), errno);
            break;
        case EC_CLI_NO_RPN_VALUE:
            printf("%s0x%x: No \"-rpn\" value provided\n\n", term_err_str.c_str(), errno);
            break;
        case EC_CLI_NO_T_VALUE:
            printf("%s0x%x: No \"-t\" value provided\n\n", term_err_str.c_str(), errno);
            break;
        case EC_CLI_UNKNOWN_OPTION:
            printf("%s0x%x: Unknown CLI option provided\n\n", term_err_str.c_str(), errno);
            break;
        default:
            printf("%s0x%x: Unknown error code\n\n", term_err_str.c_str(), errno);
            break;
    }
}

/*
 * Source: https://codereview.stackexchange.com/questions/268377/time-duration-conversion-with-units
 * Solution: Toby Speight
 * @brief Convert CLI input string into CPP std::chrono::duration expression
 */
std::chrono::milliseconds parseDuration(const std::string& num, const std::string& unit,
                                        const std::chrono::milliseconds& default_duration) {
    static const std::unordered_map<std::string, double> suffix
        { {"ms", 1},
          {"s", 1000},
          {"m", 60*1000},
          {"h", 60*60*1000} };

    try {
        auto n{std::stod(num)};
        if (n < 0) {
            n = -n;
        }
        return std::chrono::milliseconds{static_cast<unsigned long>(n * suffix.at(unit))};
    }
    catch (const std::invalid_argument&) {
        std::cerr << "ERROR: could not convert " << num << " to numeric value\n";
    }
    catch (const std::out_of_range&) {
        std::cerr << "ERROR: " << unit << " is not one of ms,s,m,h\n";
    }
    return default_duration;
}

int main(int argc, char *argv[])
{
    /* local param */
            errno   = 0;    /* error number */
    int     i       = 0;    /* loop index */
    char   *endptr;         /* string to int conversion pointer */

    /* initialize */
    errno = setvbuf(stdout, 0, _IONBF, 0);
    if (errno != EXIT_SUCCESS) {
        perror("setvbuf");
        printf("errno = %d -> %s\n", errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    /* parse CLI arguments */
    for (i=1; (i<argc) && (errno==0); i++) {
        /* check for optional arguments */
        if (strcmp(argv[i], "-pi") == 0) {
            i++;
            if (i < argc) {
                pub_interval = parseDuration(argv[i], "ms", std::chrono::milliseconds{pub_interval});
            } else {
                errno = EC_CLI_NO_PI_VALUE;
            }
        } else if (strcmp(argv[i], "-si") == 0) {
            i++;
            if (i < argc) {
                sub_interval = parseDuration(argv[i], "s", std::chrono::milliseconds{sub_interval});
            } else {
                errno = EC_CLI_NO_SI_VALUE;
            }
        } else if (strcmp(argv[i], "-cpn") == 0) {
            i++;
            if (i < argc) {
                consecutive_publish_num = strtol(argv[i], &endptr, 10);
                if (errno != EXIT_SUCCESS) {
                    perror("strtol");
                    errno = EC_CLI_INVALID_CPN_VALUE;
                }
                if (endptr == argv[i]) {
                    std::cout << "No digits were found for option \"-cpn\"\n" << std::endl;
                    errno = EC_CLI_INVALID_CPN_VALUE;
                }
            } else {
                errno = EC_CLI_NO_CPN_VALUE;
            }
        } else if (strcmp(argv[i], "-rpn") == 0) {
            i++;
            if (i < argc) {
                repeat_publish_num = strtol(argv[i], &endptr, 10);
                if (errno != EXIT_SUCCESS) {
                    perror("strtol");
                    errno = EC_CLI_INVALID_RPN_VALUE;
                }
                if (endptr == argv[i]) {
                    std::cout << "No digits were found for option \"-rpn\"\n" << std::endl;
                    errno = EC_CLI_INVALID_RPN_VALUE;
                }
            } else {
                errno = EC_CLI_NO_RPN_VALUE;
            }
        } else if (strcmp(argv[i], "-t") == 0) {
            i++;
            if (i < argc) {
                pub_sub_topic = argv[i];
            } else {
                errno = EC_CLI_NO_T_VALUE;
            }
        /* check for optional single arguments */
        } else if (strcmp(argv[i], "-b") == 0) {
            basic_mode = true;
        } else if ((strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "--help") == 0)) {
            print_help_message();
            exit(EXIT_SUCCESS);
        } else {
            errno = EC_CLI_UNKNOWN_OPTION;
        }
    }
    /* exit if CLI parsing error */
    if (errno != EXIT_SUCCESS) {
        printf("errno = %d -> %s\n", errno, strerror(errno));
        print_custom_errno_message(errno);
        print_help_message();
        exit(EXIT_FAILURE);
    } else {
        std::cout << "Arguments value:" << std::endl;
        std::cout << "  -pi \t" << pub_interval.count() << std::endl;
        std::cout << "  -si \t" << sub_interval.count() << std::endl;
        std::cout << "  -cpn\t" << consecutive_publish_num << std::endl;
        std::cout << "  -rpn\t" << repeat_publish_num << std::endl;
        std::cout << "  -t  \t" << pub_sub_topic << std::endl;
        std::cout << "  -b  \t" << basic_mode << std::endl;
    }

    /* display command format */
    std::cout << "Program starting ..." << std::endl;
    std::cout << "Command format: <step>,<angle>" << std::endl;
    std::cout << "Example: 1,90" << std::endl;

    /* main loop */
    if (basic_mode) {
        while (true) {
            publish_once(argc, argv);
        }
    } else {
        while (true) {
            std::cout << "Waiting for ready signal ..." << std::endl;
            subscribe_ready(argc, argv); // wait for ready signal
            std::cout << "Ready signal received, start publishing until terminiate signal ..." << std::endl;
            while (ready) {
                publish_once(argc, argv);
                subscribe_term(argc, argv); // wait for term signal
            }
        }
    }
    return 0;
}
