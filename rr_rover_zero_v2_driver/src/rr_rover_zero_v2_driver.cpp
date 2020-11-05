// -*- mode:c++; fill-column: 100; -*-

#include "rr_rover_zero_v2_driver/rr_rover_zero_v2_driver.h"

#include <cassert>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>
#include <rr_rover_zero_v2_driver_msgs/VescStateStamped.h>

const double wheel_base = 0.358775;
const double wheel_radius = 0.127;

namespace rr_rover_zero_v2_driver
{

  Rr_Rover_ZERO_V2_Driver::Rr_Rover_ZERO_V2_Driver(ros::NodeHandle nh,
                         ros::NodeHandle private_nh) : vesc_(std::string(),
                                                             boost::bind(&Rr_Rover_ZERO_V2_Driver::vescPacketCallback, this, _1),
                                                             boost::bind(&Rr_Rover_ZERO_V2_Driver::vescErrorCallback, this, _1)),
                                                       driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1),
                                                       e_stop_on_(false)
  {
    // get vesc serial port address
    std::string port;
    if (!private_nh.getParam("port", port))
    {
      ROS_FATAL("VESC communication port parameter required.");
      ros::shutdown();
      return;
    }

    // attempt to connect to the serial port
    try
    {
      vesc_.connect(port);
    }
    catch (SerialException e)
    {
      ROS_FATAL("Failed to connect to the VESC, %s.", e.what());
      ros::shutdown();
      return;
    }

    // create vesc state (telemetry) publisher
    state_pub_ = nh.advertise<rr_rover_zero_v2_driver_msgs::VescStateStamped>("sensors_1", 1);
    state2_pub_ = nh.advertise<rr_rover_zero_v2_driver_msgs::VescStateStamped>("sensors_2", 1);
    e_stop_sub = nh.subscribe("/soft_estop/enable", 1, &Rr_Rover_ZERO_V2_Driver::eStopCB, this);
    e_stop_reset_sub = nh.subscribe("/soft_estop/reset", 1, &Rr_Rover_ZERO_V2_Driver::eStopResetCB, this);
    twist_sub_ = nh.subscribe("/cmd_vel/", 1, &Rr_Rover_ZERO_V2_Driver::callbackTwist, this);

    // create a 50Hz timer, used for state machine & polling VESC telemetry
    timer_ = nh.createTimer(ros::Duration(1.0 / 50.0), &Rr_Rover_ZERO_V2_Driver::timerCallback, this);
  }

  void Rr_Rover_ZERO_V2_Driver::timerCallback(const ros::TimerEvent &event)
  {
    // VESC interface should not unexpectedly disconnect, but test for it anyway
    if (!vesc_.isConnected())
    {
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
    if (driver_mode_ == MODE_INITIALIZING)
    {
      // request version number, return packet will update the internal version numbers
      vesc_.requestFWVersion();
      if (fw_version_major_ >= 0 && fw_version_minor_ >= 0)
      {
        ROS_INFO("Connected to VESC with firmware version %d.%d",
                 fw_version_major_, fw_version_minor_);
        driver_mode_ = MODE_OPERATING;
      }
    }
    else if (driver_mode_ == MODE_OPERATING)
    {
      // poll for vesc state (telemetry)
      if (id == 0)
      {
        vesc_.requestState();
        id = 1;
      }
      else if (id == 1)
      {
        vesc_.requestState(8);
        id = 0;
      }
    }
    else
    {
      // unknown mode, how did that happen?
      assert(false && "unknown driver mode");
    }
  }

  void Rr_Rover_ZERO_V2_Driver::vescPacketCallback(const boost::shared_ptr<VescPacket const> &packet)
  {
    if (packet->name() == "Values")
    {
      boost::shared_ptr<VescPacketValues const> values =
          boost::dynamic_pointer_cast<VescPacketValues const>(packet);
      //TODO: Add other missing params
      rr_rover_zero_v2_driver_msgs::VescStateStamped::Ptr state_msg(new rr_rover_zero_v2_driver_msgs::VescStateStamped);
      //TODO: fix variable naming!!!!!
      state_msg->header.stamp = ros::Time::now();
      state_msg->state.motorid = id;
      state_msg->state.temp_mos = values->temp_mos();
      state_msg->state.temp_motor = values->temp_motor();
      state_msg->state.current_motor = values->current_motor();
      state_msg->state.current_input = values->current_in();
      state_msg->state.id = values->id();
      state_msg->state.iq = values->iq();
      state_msg->state.duty_now = values->duty_now();
      state_msg->state.rpm = values->rpm();
      state_msg->state.v_in = values->v_in();
      state_msg->state.amp_hours = values->amp_hours();
      state_msg->state.amp_hours_charged = values->amp_hours_charged();
      state_msg->state.watt_hours = values->watt_hours();
      state_msg->state.watt_hours_charged = values->watt_hours_charged();
      state_msg->state.tachometer = values->tachometer();
      state_msg->state.tachometer_abs = values->tachometer_abs();
      state_msg->state.fault_code = values->fault_code();
      state_msg->state.position = values->position();
      state_msg->state.vesc_id = values->vesc_id();
      state_msg->state.temp_mos_1 = values->temp_mos_1();
      state_msg->state.temp_mos_2 = values->temp_mos_2();
      state_msg->state.temp_mos_3 = values->temp_mos_3();
      state_msg->state.vd = values->vd();
      state_msg->state.vq = values->vq();

      // state_msg->state.displacement = values->watt_hours();
      // state_msg->state.distance_traveled = values->watt_hours_charged();
      if (id == 0)
      {
        state_pub_.publish(state_msg);
      }
      else if (id == 1)
      {
        state2_pub_.publish(state_msg);
      }
    }
    else if (packet->name() == "FWVersion")
    {
      boost::shared_ptr<VescPacketFWVersion const> fw_version =
          boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
      // TODO: might need lock here
      fw_version_major_ = fw_version->fwMajor();
      fw_version_minor_ = fw_version->fwMinor();
    }
  }

  void Rr_Rover_ZERO_V2_Driver::vescErrorCallback(const std::string &error)
  {
    ROS_ERROR("%s", error.c_str());
  }

  void Rr_Rover_ZERO_V2_Driver::callbackTwist(const geometry_msgs::Twist &msg)
  {
    static bool prev_e_stop_state_ = false;
    if (driver_mode_ == MODE_OPERATING)
    {
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
        vesc_.setDutyCycle(clip(msg.linear.x + 0.5 * msg.angular.z, -0.5, 0.5), 8);
      }
    }
  }

  float Rr_Rover_ZERO_V2_Driver::clip(float n, float lower, float upper)
  {
    return std::max(lower, std::min(n, upper));
  }

  void Rr_Rover_ZERO_V2_Driver::eStopCB(const std_msgs::Bool::ConstPtr &msg)
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
  void Rr_Rover_ZERO_V2_Driver::eStopResetCB(const std_msgs::Bool::ConstPtr &msg)
  {
    if (msg->data)
    {
      e_stop_on_ = false;
    }
    return;
  }

} // namespace rr_rover_zero_v2_driver
