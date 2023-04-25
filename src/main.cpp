//
// Created by yezi on 23-4-24.
//

#include "rm_auto_aim_test_jig/test_jig.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm_auto_aim_test_jig");
  ros::NodeHandle nh("~");
  auto* test_jig_manual = new rm_auto_aim_test_jig::AutoAimTestJigManual(nh);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    test_jig_manual->run();
    loop_rate.sleep();
  }
  return 0;
}
