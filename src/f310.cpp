#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <SDL/SDL.h>

class ControllerPublisher : public rclcpp::Node {
public:
    ControllerPublisher() : Node("controller_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("velocity", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControllerPublisher::timer_callback, this)
        );

        if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
            RCLCPP_ERROR(this->get_logger(), "SDL could not initialize! SDL Error: %s", SDL_GetError());
            rclcpp::shutdown();
            throw std::runtime_error("Failed to initialize SDL");
        } else {
            if (SDL_NumJoysticks() < 1) {
                RCLCPP_ERROR(this->get_logger(), "No joysticks connected!");
            } else {
                joystick_ = SDL_JoystickOpen(0);
                if (joystick_ == nullptr) {
                    RCLCPP_ERROR(this->get_logger(), "Unable to open joystick: %s", SDL_GetError());
                } else {
                    RCLCPP_INFO(this->get_logger(), "Joystick opened successfully.");
                }
            }
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
        // SDLのジョイスティックの状態を更新
        SDL_JoystickUpdate();
        if (joystick_ != nullptr) {
            int r_y = scaleAxis(SDL_JoystickGetAxis(joystick_, 4));
            int l_y = scaleAxis(SDL_JoystickGetAxis(joystick_, 1));
            std_msgs::msg::Int32MultiArray message;
            message.data = {r_y, l_y};
            RCLCPP_INFO(this->get_logger(), "Right : %d, Left : %d", r_y, l_y);
            publisher_->publish(message);
        }
    }

    int scaleAxis(int value) {
        // 軸の値を -100 から 100 の範囲にスケーリングする
        // デッドゾーンも考慮する
        const int DEAD_ZONE = 8000; // デッドゾーンの閾値
        if (std::abs(value) < DEAD_ZONE) return 0;
        return (value * 100) / 32768;
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
