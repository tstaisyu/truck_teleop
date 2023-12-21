/* Author: Taisyu Shibata */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <pigpio.h>
#include <iostream>
#include <chrono>
#include <thread>

#define bottom 50
#define R 12
#define L 13
#define ENABLE_r 17
#define ENABLE_l 18

class SubscriberNode : public rclcpp::Node 
{
public:
    SubscriberNode() : Node("subscriber"), joy_r(0), joy_l(0)
    {
    int initResult = gpioInitialise();
    if (initResult < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize GPIO: %d", initResult);
        rclcpp::shutdown();
        throw std::runtime_error("Failed to initialize GPIO");
    }

        gpioSetMode(R, PI_OUTPUT);
        gpioSetMode(L, PI_OUTPUT);
        gpioSetMode(ENABLE_r, PI_OUTPUT);
        gpioSetMode(ENABLE_l, PI_OUTPUT);

        gpioWrite(ENABLE_r, PI_LOW);
        gpioWrite(ENABLE_l, PI_LOW);

        // GPIO PWM setup code here

        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "velocity", 10, std::bind(&SubscriberNode::toGpio, this, std::placeholders::_1));
    }

    ~SubscriberNode() {
        gpioTerminate(); // Clean up pigpio library
    }

private:
    void toGpio(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        joy_r = msg->data[0];
        joy_l = msg->data[1];
        // GPIO操作のロジックをここに実装します
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    int joy_r, joy_l;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<SubscriberNode>();
        rclcpp::spin(node);
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
