#include "ros/ros.h"

#include <fcntl.h>
#include <termios.h>
#include <ctime>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Bool.h>
#include "nav_msgs/Odometry.h"

namespace canrover
{
class CanRover
{
public:
  CanRover(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
  bool start();
  bool verifyParams();
  void serialManager();
  bool e_stop_on_;
  float clip(float n, float lower, float upper);

private:
  // ROS Parameters
  std::string device_;
  std::string drive_type_;

  float timeout_;  // Default to neutral motor values after timeout seconds

  // ROS node handlers
  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_priv_;

  ros::Subscriber cmd_vel_sub;
  ros::Subscriber e_stop_sub;
  ros::Subscriber e_stop_reset_sub;

  const int LEFT_MOTOR_ID_;
  const int RIGHT_MOTOR_ID_;
  const int MOTOR_NEUTRAL = 0;
  int motor_speed_linear_coef_;
  int motor_speed_angular_coef_;
  int motor_speed_flipper_coef_;
  int motor_speed_deadband_;

  // int motor_speed_diff_max_; ---WIP
  geometry_msgs::Twist cmd_vel_commanded_;

  // ROS Subscriber callback functions
  void cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);
  void eStopCB(const std_msgs::Bool::ConstPtr& msg);
  void eStopResetCB(const std_msgs::Bool::ConstPtr& msg);

  // CAN Com Functions
  int CanSetDuty(int MotorID, float Duty);

};

}  // namespace canrover