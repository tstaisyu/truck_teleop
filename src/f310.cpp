#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <SDL.h>

class ControllerPublisher : public rclcpp::Node {
public:
    ControllerPublisher() : Node("controller_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("velocity", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControllerPublisher::timer_callback, this)
        );

        SDL_Init(SDL_INIT_JOYSTICK);
        joystick_ = SDL_JoystickOpen(0);
        if (joystick_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Joystick not found.");
        }
    }

    ~ControllerPublisher() {
        if (joystick_ != nullptr) {
            SDL_JoystickClose(joystick_);
        }
        SDL_Quit();
    }

private:
    void timer_callback() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            // Handle events here if needed
        }

        std_msgs::msg::Int32MultiArray message;
        if (joystick_ != nullptr) {
            int r_y = SDL_JoystickGetAxis(joystick_, 4) * -100;
            int l_y = SDL_JoystickGetAxis(joystick_, 1) * -100;
            message.data = {r_y, l_y};
            RCLCPP_INFO(this->get_logger(), "Right : %d, Left : %d", r_y, l_y);
            publisher_->publish(message);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    SDL_Joystick* joystick_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerPublisher>());
    rclcpp::shutdown();
    return 0;
}
