//
// Created by yezi on 23-4-24.
//

#include "rm_auto_aim_test_jig/test_jig.h"

namespace rm_auto_aim_test_jig
{
AutoAimTestJigManual::AutoAimTestJigManual(ros::NodeHandle& nh) : controller_manager_(nh)
{
  ros::NodeHandle nh_move = ros::NodeHandle(nh, "move");
  ros::NodeHandle nh_top = ros::NodeHandle(nh, "top");
  std::string move_joint_name, top_joint_name;
  nh_move.param("move_scale", move_scale_, 1.);
  nh_move.getParam("joint", move_joint_name);
  nh_top.param("top_scale", top_scale_, 1.);
  nh_top.getParam("joint", top_joint_name);
  urdf::Model urdf;
  urdf.initParamWithNodeHandle("robot_description", nh);
  move_joint_urdf_ = urdf.getJoint(move_joint_name);
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("move_calibration", rpc_value);
  move_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  move_sender_ = new rm_common::JointPointCommandSender(nh_move, joint_state_);
  top_sender_ = new rm_common::JointPointCommandSender(nh_top, joint_state_);
  dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &AutoAimTestJigManual::dbusDataCallback, this);
  joint_state_sub_ =
      nh.subscribe<sensor_msgs::JointState>("/joint_state", 10, &AutoAimTestJigManual::jointStateCallback, this);
  controller_manager_.startStateControllers();
}

void AutoAimTestJigManual::run()
{
  move_calibration_->update(ros::Time::now());
}

void AutoAimTestJigManual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  double move_vel = 0., top_vel = 0.;
  if (ros::Time::now() - data->stamp < ros::Duration(0.2))
  {
    if (data->s_r == rm_msgs::DbusData::DOWN)
      move_calibration_->reset();
    else if (data->s_r == rm_msgs::DbusData::MID)
    {
      move_vel = data->ch_r_x * move_scale_;
      top_vel = data->ch_l_x * top_scale_;
    }
    else if (data->s_r == rm_msgs::DbusData::UP)
    {
      auto_move_vel_ += data->ch_r_x;
      auto_top_vel_ += data->ch_l_x;
      if (auto_move_vel_ > move_scale_)
        auto_move_vel_ = move_scale_;
      if (auto_move_vel_ < 0.)
        auto_move_vel_ = 0.;
      if (auto_top_vel_ > top_scale_)
        auto_top_vel_ = top_scale_;
      if (auto_top_vel_ < -top_scale_)
        auto_top_vel_ = -top_scale_;
      top_vel = auto_top_vel_;
      if (std::abs(move_joint_urdf_->safety->soft_upper_limit - joint_state_.position[move_sender_->getIndex()]) < 0.1)
        move_vel = -auto_move_vel_;
      else if (std::abs(move_joint_urdf_->safety->soft_lower_limit - joint_state_.position[move_sender_->getIndex()]) <
               0.1)
        move_vel = auto_move_vel_;
    }
  }
  else
  {
    move_vel = 0;
    top_vel = 0.;
  }
  move_sender_->setPoint(move_vel);
  top_sender_->setPoint(top_vel);
  move_sender_->sendCommand(ros::Time(0));
  top_sender_->sendCommand(ros::Time(0));
}

void AutoAimTestJigManual::jointStateCallback(const sensor_msgs::JointState::ConstPtr& data)
{
  joint_state_ = *data;
}
}  // namespace rm_auto_aim_test_jig
