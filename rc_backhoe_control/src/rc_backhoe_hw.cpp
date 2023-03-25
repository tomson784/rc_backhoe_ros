#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <iostream> // for debug
#include <math.h>
#include "rc_backhoe_control/rc_backhoe_hw.h"

RcBackhoeHW::RcBackhoeHW()
{
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_1("swing_joint", &pos_[0], &vel_[0], &eff_[0]);
  jnt_state_interface.registerHandle(state_handle_1);

  hardware_interface::JointStateHandle state_handle_2("boom_joint", &pos_[1], &vel_[1], &eff_[1]);
  jnt_state_interface.registerHandle(state_handle_2);

  hardware_interface::JointStateHandle state_handle_3("arm_joint", &pos_[2], &vel_[2], &eff_[2]);
  jnt_state_interface.registerHandle(state_handle_3);

  hardware_interface::JointStateHandle state_handle_4("bucket_joint", &pos_[3], &vel_[3], &eff_[3]);
  jnt_state_interface.registerHandle(state_handle_4);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("swing_joint"), &cmd_[0]);
  jnt_pos_interface.registerHandle(pos_handle_1);

  hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("boom_joint"), &cmd_[1]);
  jnt_pos_interface.registerHandle(pos_handle_2);

  hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("arm_joint"), &cmd_[2]);
  jnt_pos_interface.registerHandle(pos_handle_3);

  hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("bucket_joint"), &cmd_[3]);
  jnt_pos_interface.registerHandle(pos_handle_4);

  registerInterface(&jnt_pos_interface);
  // std::vector<control_toolbox::Pid> pid_controllers_;

  // for (int j = 0; j < 4; j++){
  //   cmd_[j] = 0;
  // }

  // pid_controllers_.resize(4);
  // for (int j = 0; j < 4; j++){
  //   pid_controllers_[j].init(nh);
  //   control_toolbox::Pid::Gains g = pid_controllers_[j].getGains();
  //   std::cout << "p : " << g.p_gain_ << std::endl; 
  //   std::cout << "i : " << g.i_gain_ << std::endl; 
  //   std::cout << "d : " << g.d_gain_ << std::endl; 
  // }
  // pid_controllers_.resize(n_dof_);
  // if (!pid_controllers_[j].init(nh))
  //   return;
  // {

  swing_setpoint_pub = nh.advertise<std_msgs::Float64>("swing/setpoint", 100);
  boom_setpoint_pub = nh.advertise<std_msgs::Float64>("boom/setpoint", 100);
  arm_setpoint_pub = nh.advertise<std_msgs::Float64>("arm/setpoint", 100);
  bucket_setpoint_pub = nh.advertise<std_msgs::Float64>("bucket/setpoint", 100);

  // swing_state_pub = nh.advertise<std_msgs::Float64>("swing/state", 100);
  // boom_state_pub = nh.advertise<std_msgs::Float64>("boom/state", 100);
  // arm_state_pub = nh.advertise<std_msgs::Float64>("arm/state", 100);
  // bucket_state_pub = nh.advertise<std_msgs::Float64>("bucket/state", 100);
  // pub_cmd = nh.advertise<std_msgs::Float32MultiArray>("cmd", 1);
  pub_cmd = nh.advertise<std_msgs::Int8MultiArray>("cmd", 1);
  pub_debug = nh.advertise<std_msgs::Float32MultiArray>("debug", 1);
  pub_setpoints = nh.advertise<std_msgs::Float32MultiArray>("setpoints", 1);
  pub_cmd_pos = nh.advertise<std_msgs::Float32MultiArray>("cmd/pos", 1);
  pub_cmd_vel = nh.advertise<std_msgs::Float32MultiArray>("cmd/vel", 1);
  pub_cmd_eff = nh.advertise<std_msgs::Float32MultiArray>("cmd/eff", 1);

  js_sub = nh.subscribe("joint_states", 10, &RcBackhoeHW::jsCallback, this);
  // fake_sub = nh.subscribe("fake_joint_state", 10, &RcBackhoeHW::faketimeCallback, this);
  std::cout << "\x1b[46mTest" << std::endl;
}

// void RcBackhoeHW::faketimeCallback(const sensor_msgs::JointState::ConstPtr &msg_sub){
//   pos_[4] = msg_sub->position[0];
// }

void RcBackhoeHW::jsCallback(const sensor_msgs::JointState::ConstPtr &msg_sub)
{
  int msg_size = msg_sub->position.size();
  for (int i = 0; i < msg_size; i++)
  {
    if(msg_sub->name[i] == "swing_joint") {
      pos_[0] = msg_sub->position[i];
    }
    else if (msg_sub->name[i] == "boom_joint") {
      pos_[1] = msg_sub->position[i];
    }
    else if (msg_sub->name[i] == "arm_joint") {
      pos_[2] = msg_sub->position[i];
    }
    else if (msg_sub->name[i] == "bucket_joint") {
      pos_[3] = msg_sub->position[i];
    }
  }
}

void RcBackhoeHW::read(ros::Time time, ros::Duration period)
{
  // for(int i=0; i<4; i++){
  //   vel_[i] = 0;
  //   eff_[i] = 0;
  // }
}

void RcBackhoeHW::write(ros::Time time, ros::Duration period)
{
  swing_setpoint.data = cmd_[0];
  boom_setpoint.data = cmd_[1];
  arm_setpoint.data = cmd_[2];
  bucket_setpoint.data = cmd_[3];
  // std::cout << cmd_[0] << ", " << cmd_[1] << ", " << cmd_[2] << ", " << cmd_[3] << ", " << std::endl;

  swing_setpoint_pub.publish(swing_setpoint);
  boom_setpoint_pub.publish(boom_setpoint);
  arm_setpoint_pub.publish(arm_setpoint);
  bucket_setpoint_pub.publish(bucket_setpoint);

  std_msgs::Float32MultiArray setpoints_msgs;
  for (int i = 0; i < 4; i++){
    setpoints_msgs.data.push_back(cmd_[i]*180/M_PI);
  }
  pub_setpoints.publish(setpoints_msgs);
  // 
  std_msgs::Float32MultiArray cmd_pos_msgs;
  for (int i = 0; i < 4; i++){
    cmd_pos_msgs.data.push_back(pos_[i]*180/M_PI);
  }
  pub_cmd_pos.publish(cmd_pos_msgs);
  // 
  std_msgs::Float32MultiArray cmd_vel_msgs;
  for (int i = 0; i < 4; i++){
    cmd_vel_msgs.data.push_back(vel_[i]*180/M_PI);
  }
  pub_cmd_vel.publish(cmd_vel_msgs);
  // 
  std_msgs::Float32MultiArray cmd_eff_msgs;
  for (int i = 0; i < 4; i++){
    cmd_eff_msgs.data.push_back(eff_[i]);
  }
  pub_cmd_eff.publish(cmd_eff_msgs);
  // 
  std_msgs::Float32MultiArray debug_array;
  for (int i = 0; i < 4; i++){
    debug_array.data.push_back((cmd_[i] - pos_[i])*180/M_PI);
  }
  pub_debug.publish(debug_array);

  // std_msgs::Float64 tmp;
  // tmp.data = pos_[0];
  // swing_state_pub.publish(tmp);
  // tmp.data = pos_[1];
  // boom_state_pub.publish(tmp);
  // tmp.data = pos_[2];
  // arm_state_pub.publish(tmp);
  // tmp.data = pos_[3];
  // bucket_state_pub.publish(tmp);
  // pub_cmd.
  // std_msgs::Float32MultiArray cmd_msgs;
  std_msgs::Int8MultiArray cmd_msgs;
  cmd_msgs.data.resize(4);
  // const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
  //                           -effort_limit, effort_limit);
  // std::cout << "boom : " << cmd_[1]*180/3.14 << " - " << pos_[1]*180/3.14 << std::endl;

  for (int i = 0; i < 4; i++){
    int cmd = 0;
    if (cmd_[i] - pos_[i] > 0.3){ cmd = -1; }
    else if(cmd_[i] - pos_[i] < -0.3){ cmd = 1; }
    cmd_msgs.data[i] = cmd;
    // cmd_msgs.data.push_back(pos_[i] - cmd_[i]);
  }
  // std::cout << cmd_[0] - pos_[0] << ", " << cmd_[1] - pos_[1] << ", " << cmd_[2] - pos_[2] << ", " << cmd_[3] - pos_[3] << ", " << std::endl;
  pub_cmd.publish(cmd_msgs);

}
