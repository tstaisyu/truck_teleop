/* Author: Taisyu Shibata */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <JetsonGPIO.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
using std::placeholders::_1;

#define R 12
#define L 13
#define ENABLE_r 17
#define ENABLE_l 18

class SubscriberNode : public rclcpp::Node 
{
public:
    SubscriberNode() : Node("subscriber"), joy_r(0), joy_l(0)
    {
        RCLCPP_INFO(this->get_logger(), "Setting up GPIO using JetsonGPIO...");
        
        // JetsonGPIOを設定
        GPIO::setmode(GPIO::BCM);
        
        GPIO::setup(R, GPIO::OUT, GPIO::LOW);
        GPIO::setup(L, GPIO::OUT, GPIO::LOW);
        GPIO::setup(ENABLE_r, GPIO::OUT, GPIO::LOW);
        GPIO::setup(ENABLE_l, GPIO::OUT, GPIO::LOW);

        RCLCPP_INFO(this->get_logger(), "GPIO setup completed.");

        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "velocity", rclcpp::SystemDefaultsQoS(), std::bind(&SubscriberNode::ToGpio, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscription created successfully.");
    }

    ~SubscriberNode() {
        GPIO::cleanup(); // Clean up GPIO library
    }

private:
    void ToGpio(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
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
        RCLCPP_INFO(this->get_logger(), "Right Joystick: %d, Left Joystick: %d", joy_r, joy_l);

        // PWM制御
        GPIO::output(ENABLE_r, GPIO::HIGH); // 例：モーターの方向制御
        GPIO::output(ENABLE_l, GPIO::HIGH);
        GPIO::softPWMCreate(R, 0, 100); // PWM範囲の設定（0-100）
        GPIO::softPWMCreate(L, 0, 100);
        GPIO::softPWMWrite(R, joy_r); // PWM制御
        GPIO::softPWMWrite(L, joy_l);
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
