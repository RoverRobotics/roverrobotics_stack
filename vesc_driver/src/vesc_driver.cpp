// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_driver.h"

#include <cassert>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>
#include <vesc_msgs/VescStateStamped.h>

const double wheel_base = 0.358775;
const double wheel_radius = 0.127;

namespace vesc_driver
{

VescDriver::VescDriver(ros::NodeHandle nh,
                       ros::NodeHandle private_nh):
  vesc_(std::string(),
        boost::bind(&VescDriver::vescPacketCallback, this, _1),
        boost::bind(&VescDriver::vescErrorCallback, this, _1)),
  driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1),
  e_stop_on_(false)
{
  // get vesc serial port address
  std::string port;
  if (!private_nh.getParam("port", port)) {
    ROS_FATAL("VESC communication port parameter required.");
    ros::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try {
    vesc_.connect(port);
  }
  catch (SerialException e) {
    ROS_FATAL("Failed to connect to the VESC, %s.", e.what());
    ROS_INFO("Retrying Every 1 Second");
    while(!vesc_.isConnected()){
	sleep(1000);
	ROS_INFO("...");
	vesc_.connect(port);
    }
    //ros::shutdown();
    //return;
  }

  // create vesc state (telemetry) publisher
  state_pub_ = nh.advertise<vesc_msgs::VescStateStamped>("sensors/core", 10);
  e_stop_sub = nh.subscribe("/soft_estop/enable", 1, &VescDriver::eStopCB, this);
  e_stop_reset_sub = nh.subscribe("/soft_estop/reset", 1, &VescDriver::eStopResetCB, this);
  twist_sub_ = nh.subscribe("/cmd_vel/", 1, &VescDriver::callbackTwist, this);

  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh.createTimer(ros::Duration(1.0/50.0), &VescDriver::timerCallback, this);
}

void VescDriver::timerCallback(const ros::TimerEvent& event)
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      ROS_INFO("Connected to VESC with firmware version %d.%d",
               fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING) {
    // poll for vesc state (telemetry)
    vesc_.requestState();
  }
  else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriver::vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet)
{
  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
      boost::dynamic_pointer_cast<VescPacketValues const>(packet);

    vesc_msgs::VescStateStamped::Ptr state_msg(new vesc_msgs::VescStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->state.voltage_input = values->v_in();
    state_msg->state.temperature_pcb = values->temp_pcb();
    state_msg->state.current_motor = values->current_motor();
    state_msg->state.current_input = values->current_in();
    state_msg->state.speed = values->rpm();
    state_msg->state.duty_cycle = values->duty_now();
    state_msg->state.charge_drawn = values->amp_hours();
    state_msg->state.charge_regen = values->amp_hours_charged();
    state_msg->state.energy_drawn = values->watt_hours();
    state_msg->state.energy_regen = values->watt_hours_charged();
    state_msg->state.displacement = values->tachometer();
    state_msg->state.distance_traveled = values->tachometer_abs();
    state_msg->state.fault_code = values->fault_code();

    state_pub_.publish(state_msg);
  }
  else if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // TODO: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
}

void VescDriver::vescErrorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}

void VescDriver::callbackTwist(const geometry_msgs::Twist &msg) {
	static bool prev_e_stop_state_ = false;
    if (driver_mode_ == MODE_OPERATING) {	
    if (e_stop_on_)
    {
      if (!prev_e_stop_state_)
      {
        prev_e_stop_state_ = true;
        ROS_WARN("Rover driver - Soft e-stop on.");
      }
      vesc_.setDutyCycle(0);
      vesc_.setDutyCycle(0, 8);
      return;
    }
    else
    {
      if (prev_e_stop_state_)
      {
        prev_e_stop_state_ = false;
        ROS_INFO("Rover driver - Soft e-stop off.");
      }
      vesc_.setDutyCycle(clip(msg.linear.x - 0.5 * msg.angular.z, -0.5, 0.5));
      vesc_.setDutyCycle(clip(msg.linear.x + 0.5 * msg.angular.z, -0.5 , 0.5), 8);
    }	
        }
}

 float VescDriver::clip(float n, float lower, float upper)
  {
    return std::max(lower, std::min(n, upper));
  }

void VescDriver::eStopCB(const std_msgs::Bool::ConstPtr &msg)
  {
    static bool prev_e_stop_state_ = false;

    // e-stop only trigger on the rising edge of the signal and only deactivates when reset
    if (msg->data && !prev_e_stop_state_)
    {
      e_stop_on_ = true;
    }

    prev_e_stop_state_ = msg->data;
    return;
  }
  void VescDriver::eStopResetCB(const std_msgs::Bool::ConstPtr &msg)
  {
    if (msg->data)
    {
      e_stop_on_ = false;
    }
    return;
  }


} // namespace vesc_driver
