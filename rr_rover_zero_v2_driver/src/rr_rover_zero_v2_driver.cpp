// -*- mode:c++; fill-column: 100; -*-

#include "rr_rover_zero_v2_driver/rr_rover_zero_v2_driver.h"

#include <cassert>
#include <cmath>
#include <sstream>
#include "nav_msgs/Odometry.h"
#include <boost/bind.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
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
    state_pub_ = nh.advertise<rr_rover_zero_v2_driver_msgs::VescStateStamped>("driver_telemetry", 1);
    // create robot odometry publisher
    odom_enc_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
    e_stop_sub = nh.subscribe("/soft_estop/enable", 1, &Rr_Rover_ZERO_V2_Driver::eStopCB, this);
    e_stop_reset_sub = nh.subscribe("/soft_estop/reset", 1, &Rr_Rover_ZERO_V2_Driver::eStopResetCB, this);
    twist_sub_ = nh.subscribe("/cmd_vel/", 1, &Rr_Rover_ZERO_V2_Driver::callbackTwist, this);
    trim_sub_ = nh.subscribe("/trim_increment", 1, &Rr_Rover_ZERO_V2_Driver::trimCB, this);
    // create a 50Hz timer, used for state machine & polling VESC telemetry
    timer_ = nh.createTimer(ros::Duration(1.0 / 50.0), &Rr_Rover_ZERO_V2_Driver::timerCallback, this);

    //Get Parameters
    private_nh.getParam("trim", trim);
    private_nh.getParam("odom_angular_coef", odom_angular_coef_);
    private_nh.getParam("odom_traction_factor", odom_traction_factor_);
    private_nh.getParam("hall_ratio", hall_ratio_);

    ROS_INFO("got the following param values \n trim: %f \n odom_angular_coef: %f \n odom_traction_factor: %f \n hall_ratio: %f", trim, odom_angular_coef_, odom_traction_factor_, hall_ratio_);
  }

  void Rr_Rover_ZERO_V2_Driver::trimCB(const std_msgs::Float32::ConstPtr &msg)
  { //Get trim_increment value
    trim += msg->data;
    ROS_INFO("Trim value is at %f", trim);
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
      double left_vel_measured_ = 0;
      double right_vel_measured_ = 0;

      vesc_.requestState();
      left_vel_measured_ = rpm_1;
      vesc_.requestState(8);
      right_vel_measured_ = rpm_2;

      publishOdometry(left_vel_measured_ / hall_ratio_, right_vel_measured_ / hall_ratio_);
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
      if (values
              ->vesc_id() == 1)
      {
        rpm_1 = values->rpm();
      }
      else if (values
                   ->vesc_id() == 8)
      {
        rpm_2 = values->rpm();
      }
      state_pub_.publish(state_msg);
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
        double turn_rate = msg.angular.z;
        double linear_rate = msg.linear.x;
        if (turn_rate == 0)
        {
          if (linear_rate > 0.0)
          {
            turn_rate = trim;
          }
          else if (linear_rate < 0.0)
          {
            turn_rate = -trim;
          }
        }
        vesc_.setDutyCycle(clip(msg.linear.x - 0.5 * turn_rate, -0.5, 0.5) / 4);
        vesc_.setDutyCycle(clip(msg.linear.x + 0.5 * turn_rate, -0.5, 0.5) / 4, 8);
        // vesc_.setCurrent(clip())
        //vesc_.setCurrent(4);
        // vesc_.setSpeed(clip(msg.linear.x - 0.5 * turn_rate,-5,5));
        // vesc_.setSpeed(clip(msg.linear.x + 0.5 * turn_rate,-5,5) , 8);
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

  void Rr_Rover_ZERO_V2_Driver::publishOdometry(float left_vel, float right_vel)
  { // convert encoder readings to real world values and publish as Odometry
  ros:
    static double left_dist = 0;
    static double right_dist = 0;
    static double pos_x = 0;
    static double pos_y = 0;
    static double theta = 0;
    static double past_time = 0;
    double net_vel = 0;
    double diff_vel = 0;
    double alpha = 0;
    double dt = 0;

    tf2::Quaternion q_new;

    ros::Time ros_now_time = ros::Time::now();
    double now_time = ros_now_time.toSec();
    tf2_ros::TransformBroadcaster odom_broadcaster;
    nav_msgs::Odometry odom_msg;

    dt = now_time - past_time;
    past_time = now_time;

    if (past_time != 0)
    {
      net_vel = 0.5 * (left_vel + right_vel);
      diff_vel = right_vel - left_vel;
      alpha = odom_angular_coef_ * diff_vel * odom_traction_factor_;

      pos_x = pos_x + net_vel * cos(theta) * dt;
      pos_y = pos_y + net_vel * sin(theta) * dt;
      theta = (theta + alpha) * dt;

      q_new.setRPY(0, 0, theta);
      tf2::convert(q_new, odom_msg.pose.pose.orientation);
      // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = pos_x;
      odom_trans.transform.translation.y = pos_y;
      odom_trans.transform.translation.z = 0.0;
      //odom_trans.transform.rotation = odom_quat;
      odom_trans.transform.rotation.x = q_new.x();
      odom_trans.transform.rotation.y = q_new.y();
      odom_trans.transform.rotation.z = q_new.z();
      odom_trans.transform.rotation.w = q_new.w();
      //ROS_INFO("%f\n%f\n%f\n%f\n%f\n%f\n",pos_x,pos_y,q_new.x(),q_new.y(),q_new.z(),q_new.w());
      // odom_broadcaster.sendTransform(odom_trans);
    }

    odom_msg.header.stamp = ros_now_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.twist.twist.linear.x = net_vel;
    odom_msg.twist.twist.angular.z = alpha;

    // // If not moving, trust the encoders completely
    // // otherwise set them to the ROS param
    // if (net_vel == 0 && alpha == 0)
    // {
    //   odom_msg.twist.covariance[0] = odom_covariance_0_ / 1e3;
    //   odom_msg.twist.covariance[7] = odom_covariance_0_ / 1e3;
    //   odom_msg.twist.covariance[35] = odom_covariance_35_ / 1e6;
    // }
    // else
    // {
    //   odom_msg.twist.covariance[0] = odom_covariance_0_;
    //   odom_msg.twist.covariance[7] = odom_covariance_0_;
    //   odom_msg.twist.covariance[35] = odom_covariance_35_;
    // }

    odom_msg.pose.pose.position.x = pos_x;
    odom_msg.pose.pose.position.y = pos_y;
    odom_enc_pub_.publish(odom_msg);
    ros::spinOnce();
    return;
  }

} // namespace rr_rover_zero_v2_driver
