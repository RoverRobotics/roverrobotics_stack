// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER_VESC_DRIVER_H_
#define VESC_DRIVER_VESC_DRIVER_H_

#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <boost/optional.hpp>
#include "geometry_msgs/Twist.h"

#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{

class VescDriver
{
public:

  VescDriver(ros::NodeHandle nh,
             ros::NodeHandle private_nh);
  bool e_stop_on_;
private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void vescErrorCallback(const std::string& error);

  // ROS services
  ros::Publisher state_pub_;
  ros::Publisher state2_pub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber e_stop_sub;
  ros::Subscriber e_stop_reset_sub;
  ros::Timer timer_;

  // driver modes (possible states)
  typedef enum {
    MODE_INITIALIZING,
    MODE_OPERATING
  } driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc
  int id = 0;

  // ROS callbacks
  void timerCallback(const ros::TimerEvent& event);
  void callbackTwist(const geometry_msgs::Twist &msg);
  void eStopCB(const std_msgs::Bool::ConstPtr &msg);
  void eStopResetCB(const std_msgs::Bool::ConstPtr &msg);
  float clip(float n, float lower, float upper);
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_DRIVER_H_
