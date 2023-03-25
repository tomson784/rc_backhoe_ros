#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <random>

#define random(lower, upper) (rand() * (upper - lower) / RAND_MAX + lower)


int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_commander");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Set up the arm planning interface
  moveit::planning_interface::MoveGroupInterface arm("zx35u");

  int seed(ros::Time::now().toSec());
  srand(seed);

  // Prepare
  ROS_INFO("Moving to prepare pose");
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", arm.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", arm.getEndEffectorLink().c_str());
  arm.setPlanningTime(0.1);
  arm.setPlannerId("RRTConnect");
  arm.setGoalTolerance(0.1);
  arm.setMaxAccelerationScalingFactor(0.5);
  arm.setMaxVelocityScalingFactor(0.5);
  // ::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
  // kdl_kinematics_plugin::KDLKinematicsPlugin robot_kin;
  // robot_kin.initialize();
  // std::vector<std::string> a = {"swing_joint", "boom_joint", "arm_joint", "bucket_joint"};
  // std::vector<double> joints = {0.1, 0.1, 0.1, 0.1};
  // std::vector<geometry_msgs::Pose> pose;
  // b.getPositionFK(a, joints, pose);
  // arm.

  moveit::planning_interface::MoveItErrorCode ret;

  // pose1
  geometry_msgs::PoseStamped pose1;
  pose1.header.frame_id = "swing_link";
  // position x:4.703370, y:-0.000152, z:-0.107956
  // rotation x:0.000016, y:0.956217, z:-0.000005, w:0.292660
  bool end_flag = false;
  while (!end_flag){
    pose1.pose.position.x = 4.7 + random(-0.1,0.1);
    pose1.pose.position.y = 0.0 + random(-0.1,0.1);
    pose1.pose.position.z = 0.1 + random(-0.1,0.1);
    pose1.pose.orientation.x = 0.000016 + random(-0.01,0.01);
    pose1.pose.orientation.y = 0.956217 + random(-0.01,0.01);
    pose1.pose.orientation.z = -0.000005 + random(-0.01,0.01);
    pose1.pose.orientation.w = 0.292660 + random(-0.01,0.01);


    ROS_INFO("move to WP1");
    arm.setPoseTarget(pose1);
    ret = arm.move();
    if (!ret) {
      ROS_WARN("Fail: %i", ret.val);
    }
    else{
      end_flag = true;
    }
  }

  geometry_msgs::PoseStamped pose2;
  pose2.header.frame_id = "swing_link";
  // position x:2.295724, y:0.000159, z:0.261560
  // rotation x:-0.000029, y:0.822958, z:-0.000020, w:-0.568102
  end_flag = false;
  while (!end_flag){

    pose2.pose.position.x = 2.295724 + random(-0.1,0.1);
    pose2.pose.position.y = 0.000159 + random(-0.1,0.1);
    pose2.pose.position.z = 0.261560 + random(-0.1,0.1);
    pose2.pose.orientation.x = -0.000029 + random(-0.01,0.01);
    pose2.pose.orientation.y = 0.822958 + random(-0.01,0.01);
    pose2.pose.orientation.z = -0.000020 + random(-0.01,0.01);
    pose2.pose.orientation.w = -0.568102 + random(-0.01,0.01);
    ROS_INFO("move to WP2");
    arm.setPoseTarget(pose2);
    ret = arm.move();
    if (!ret) {
        ROS_WARN("Fail: %i", ret.val);
      }
      else{
        end_flag = true;
      }
  }
  
  ros::Duration(0.5).sleep();

  // ROS_INFO("move to WP3");
  // arm.setPoseTarget(pose3);
  // ret = arm.move();
  // if (!ret) {
  //   ROS_WARN("Fail: %i", ret.val);
  // }
  // ros::Duration(0.5).sleep();

  // ROS_INFO("move to WP4");
  // arm.setPoseTarget(pose4);
  // ret = arm.move();
  // if (!ret) {
  //   ROS_WARN("Fail: %i", ret.val);
  // }
  // ros::Duration(0.5).sleep();

  ros::shutdown();
  return 0;
}