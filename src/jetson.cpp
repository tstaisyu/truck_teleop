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
#define ENABLE_r 17
#define ENABLE_l 32

class SubscriberNode : public rclcpp::Node 
{
public:
    SubscriberNode() : Node("subscriber"), joy_r(0), joy_l(0), running(true)
    {
        RCLCPP_INFO(this->get_logger(), "Setting up GPIO using JetsonGPIO...");
        
        // JetsonGPIOを設定
        GPIO::setmode(GPIO::BCM);
        
        GPIO::setup(R, GPIO::OUT, GPIO::LOW);
        GPIO::setup(L, GPIO::OUT, GPIO::LOW);
        GPIO::setup(ENABLE_r, GPIO::OUT, GPIO::LOW);
        GPIO::setup(ENABLE_l, GPIO::OUT, GPIO::LOW);

        // PWM制御のためのスレッドを開始
        pwm_thread_r = std::thread([this] { pwm_loop(R, &joy_r); });
        pwm_thread_l = std::thread([this] { pwm_loop(L, &joy_l); });


        RCLCPP_INFO(this->get_logger(), "GPIO setup completed.");

        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "velocity", rclcpp::SystemDefaultsQoS(), std::bind(&SubscriberNode::ToGpio, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscription created successfully.");
    }

    ~SubscriberNode() {
        // PWMスレッドの停止
        running = false;
        if (pwm_thread_r.joinable()) pwm_thread_r.join();
        if (pwm_thread_l.joinable()) pwm_thread_l.join();

        GPIO::cleanup(); // Clean up GPIO library
    }

private:
    void pwm_loop(int pin, std::atomic<int>* duty_cycle) {
        while (running) {
            if ((*duty_cycle) > 0) {
                GPIO::output(pin, GPIO::HIGH);
                std::this_thread::sleep_for(std::chrono::milliseconds((*duty_cycle)));
            }
            if ((*duty_cycle) < 100) {
                GPIO::output(pin, GPIO::LOW);
                std::this_thread::sleep_for(std::chrono::milliseconds(100 - (*duty_cycle)));
            }
        }
    }

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

        // std::atomic<int> オブジェクトから値を読み取る
        int right_joystick = joy_r.load();
        int left_joystick = joy_l.load();

        RCLCPP_INFO(this->get_logger(), "Right Joystick: %d, Left Joystick: %d", right_joystick, left_joystick);

    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    std::atomic<int> joy_r, joy_l;
    std::thread pwm_thread_r, pwm_thread_l;
    std::atomic<bool> running;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
