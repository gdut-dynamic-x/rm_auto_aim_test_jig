//
// Created by yezi on 23-4-24.
//

#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <rm_msgs/DbusData.h>

namespace rm_auto_aim_test_jig
{
class AutoAimTestJigManual
{
public:
  void run();

private:
  void dbusDataCallbck(const rm_msgs::DbusData::ConstPtr& data);
  ros::Publisher top_cmd_pub_, move_cmd_pub_;
  ros::Subscriber dbus_sub_;
};
}  // namespace rm_auto_aim_test_jig
