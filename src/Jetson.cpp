/* Author: Taisyu Shibata */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <JetsonGPIO.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <atomic>
using std::placeholders::_1;

#define R 15
#define L 18
#define ENABLE_r 13
#define ENABLE_l 16

// JetsonGPIOを設定
GPIO::setmode(GPIO::BOARD);       
GPIO::setup(R, GPIO::OUT, GPIO::LOW);
GPIO::setup(L, GPIO::OUT, GPIO::LOW);
GPIO::setup(ENABLE_r, GPIO::OUT, GPIO::LOW);
GPIO::setup(ENABLE_l, GPIO::OUT, GPIO::LOW);
GPIO::PWM PWM_R(R, 50);
GPIO::PWM PWM_L(L, 50);
auto val = 25.0;
PWM_R.start(val);
PWM_L.start(val);

class SubscriberNode : public rclcpp::Node 
{
public:
    SubscriberNode() : Node("subscriber"), joy_r(0), joy_l(0), running(true)
    {
        RCLCPP_INFO(this->get_logger(), "GPIO setup completed.");

        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "velocity", rclcpp::SystemDefaultsQoS(), std::bind(&SubscriberNode::ToGpio, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscription created successfully.");
    }

    ~SubscriberNode() {
        PWM_R.stop();
        PWM_L.stop();
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

        int joy_r = msg->data[0];
        int joy_l = msg->data[1];

        PWM_R.ChangeDutyCycle(joy_r);
        PWM_L.ChangeDutyCycle(joy_l);
        
        RCLCPP_INFO(this->get_logger(), "Right Joystick: %d, Left Joystick: %d", right_joystick, left_joystick);

    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
