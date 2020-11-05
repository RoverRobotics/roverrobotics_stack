#include <ros/ros.h>

#include "rr_rover_zero_v2_driver/rr_rover_zero_v2_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rr_rover_zero_v2_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  rr_rover_zero_v2_driver::Rr_Rover_ZERO_V2_Driver rr_rover_zero_v2_driver(nh, private_nh);

  ros::spin();

  return 0;
}
