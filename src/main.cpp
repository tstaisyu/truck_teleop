/* Author: Taisyu Shibata */

#include "rclcpp/rclcpp.hpp"
#include <pigpio.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
//  rclcpp::spin();
  rclcpp::shutdown();
  return 0;
}
