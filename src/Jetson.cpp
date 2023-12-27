/* Author: Taisyu Shibata */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <iostream>
#include <chrono>
#include <string>
#include <thread>
#include <signal.h>
#include <JetsonGPIO.h>
using std::placeholders::_1;

#define R 15
#define L 18
#define ENABLE_r 13
#define ENABLE_l 16

inline void delay(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

static bool end_this_program = false;

void signalHandler(int /*s*/) { end_this_program = true; }

class SubscriberNode : public rclcpp::Node 
{
public:
    SubscriberNode() : Node("subscriber"), PWM_R(R, 50), PWM_L(L, 50), joy_r(40), joy_l(40)
    {

        // JetsonGPIOを設定
        GPIO::setup(R, GPIO::OUT, GPIO::LOW);
        GPIO::setup(L, GPIO::OUT, GPIO::LOW);
        GPIO::setup(ENABLE_r, GPIO::OUT, GPIO::LOW);
        GPIO::setup(ENABLE_l, GPIO::OUT, GPIO::LOW);

        PWM_R.start(0);
        PWM_L.start(0);

        RCLCPP_INFO(this->get_logger(), "GPIO setup completed.");

        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "velocity", rclcpp::SystemDefaultsQoS(), std::bind(&SubscriberNode::ToGpio, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscription created successfully.");
    }

    ~SubscriberNode() {
        PWM_R.stop();
        PWM_L.stop();
    }

private:
    void ToGpio(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        delay(0.2);

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

        duty_cycle_R = joy_r;
        duty_cycle_L = joy_l;
        PWM_R.ChangeDutyCycle(joy_r);
        PWM_L.ChangeDutyCycle(joy_l);
        
        RCLCPP_INFO(this->get_logger(), "Right Joystick: %d, Right Duty Cycle: %f", joy_r, duty_cycle_R);
        RCLCPP_INFO(this->get_logger(), "Left Joystick: %d, Left Duty Cycle: %f", joy_l, duty_cycle_L);

    }

    // メンバ変数
    GPIO::PWM PWM_R;
    GPIO::PWM PWM_L;
    int joy_r;
    int joy_l;
    double duty_cycle_R;
    double duty_cycle_L;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
    GPIO::setmode(GPIO::BOARD);     
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    GPIO::cleanup(); // Clean up GPIO library
    return 0;
}
