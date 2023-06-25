//
// Created by yezi on 23-4-24.
//

#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <rm_msgs/DbusData.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/calibration_queue.h>

namespace rm_auto_aim_test_jig
{
class AutoAimTestJigManual
{
public:
  AutoAimTestJigManual(ros::NodeHandle& nh);
  void run();

private:
  enum class STATE
  {
    IDLE,
    NORMAL
  } state_;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& data);
  urdf::JointConstSharedPtr move_joint_urdf_;
  sensor_msgs::JointState joint_state_;
  double move_scale_, top_scale_;
  double auto_move_vel_ = 0., auto_top_vel_ = 0.;
  int auto_move_direction_ = 1;
  bool remote_is_open_ = false;
  rm_common::JointPointCommandSender *top_sender_, *move_sender_;
  rm_common::CalibrationQueue* move_calibration_;
  rm_common::ControllerManager controller_manager_;
  ros::Subscriber dbus_sub_, joint_state_sub_;
};
}  // namespace rm_auto_aim_test_jig
