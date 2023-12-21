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
        RCLCPP_INFO(this->get_logger(), "Initializing pigpio library...");
        if (gpioInitialise() < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio library");
            rclcpp::shutdown();
            throw std::runtime_error("Failed to initialize GPIO");
        }
        RCLCPP_INFO(this->get_logger(), "Successfully initialized pigpio library.");

        gpioSetMode(R, PI_OUTPUT);
        gpioSetMode(L, PI_OUTPUT);
        gpioSetMode(ENABLE_r, PI_OUTPUT);
        gpioSetMode(ENABLE_l, PI_OUTPUT);

        gpioWrite(ENABLE_r, PI_LOW);
        gpioWrite(ENABLE_l, PI_LOW);

        // GPIO PWM setup code here (if needed)

        RCLCPP_INFO(this->get_logger(), "GPIO set up completed.");

        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "velocity", rclcpp::SystemDefaultsQoS(), std::bind(&SubscriberNode::toGpio, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscription created successfully.");
    }

    ~SubscriberNode() {
        gpioTerminate(); // Clean up pigpio library
    }

private:
    void toGpio(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "toGpio callback called.");
        if (!msg) {
            RCLCPP_ERROR(this->get_logger(), "Received null pointer in callback");
            return;
        }

        if (msg->data.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Invalid joystick data size: %zu", msg->data.size());
            return;
        }

        joy_r = msg->data[0];
        joy_l = msg->data[1];

        // Implement GPIO operations logic here
        RCLCPP_INFO(this->get_logger(), "Right Joystick: %d, Left Joystick: %d", joy_r, joy_l);
        
        // Example: Set GPIO pin values based on joystick input
        // gpioPWM(R, joy_r); // Adjust as necessary
        // gpioPWM(L, joy_l); // Adjust as necessary
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    int joy_r, joy_l;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
