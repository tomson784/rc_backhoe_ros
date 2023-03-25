#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "rc_backhoe_control/rc_backhoe_hw.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rc_backhoe_control");
  ros::NodeHandle nh;

  RcBackhoeHW rc_backhoe_hw;
  controller_manager::ControllerManager cm(&rc_backhoe_hw, rc_backhoe_hw.nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    ros::Time now = rc_backhoe_hw.getTime();
    ros::Duration dt = rc_backhoe_hw.getPeriod();

    rc_backhoe_hw.read(now, dt);
    cm.update(now, dt);

    rc_backhoe_hw.write(now, dt);
    dt.sleep();
  }
  spinner.stop();

  return 0;
}

