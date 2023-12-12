/* Author: Taisyu Shibata */

#include <ros/ros.h>

#include "raspi_on_truck.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "raspi_on_truck");

    ros::spin();
    return(0);
}
