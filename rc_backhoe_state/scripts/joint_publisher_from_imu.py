#!/usr/bin/env python
# -*- coding: utf-8 -*-

##
# @file 
# @brief 
#

import rospy
import rospkg
import numpy as np
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState, Imu
# import tf_conversions
import copy

class BackhoeJointStatus():
    def __init__(self):
        rospy.init_node('joint_status')
        # param list
        self.hz = rospy.get_param('~hz', 30.0)
        self.omega = rospy.get_param('~omega', 0.05)
        # 回転軸
        self.boom_imu_rot_axis = rospy.get_param('~boom/axis', "z")
        self.arm_imu_rot_axis = rospy.get_param('~arm/axis', "z")
        self.bucket_imu_rot_axis = rospy.get_param('~bucket/axis', "z")
        self.swing_imu_rot_axis = rospy.get_param('~swing/axis', "z")
        # offset
        self.init_boom_ang_offset = np.deg2rad(-136.0905+5)
        self.init_arm_ang_offset = np.deg2rad(307.7103-10)
        self.init_bucket_ang_offset = np.deg2rad(-19.3603+4)
        # self.init_boom_ang_offset = np.deg2rad(-136.0905+5)
        # self.init_arm_ang_offset = np.deg2rad(307.7103-10)
        # self.init_bucket_ang_offset = np.deg2rad(-19.3603+4)
        # self.init_boom_ang_offset = 0
        # self.init_arm_ang_offset = 0
        # self.init_bucket_ang_offset = 0

        # publisher list
        self.pub_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)

        # subscriber list
        rospy.Subscriber("boom/imu/data", Imu, self.cb_boom_imu)
        rospy.Subscriber("arm/imu/data", Imu, self.cb_arm_imu)
        rospy.Subscriber("bucket/imu/data", Imu, self.cb_bucket_imu)
        rospy.Subscriber("swing/imu/data", Imu, self.cb_swing_imu)
        self.boom_imu = Imu()
        self.arm_imu = Imu()
        self.bucket_imu = Imu()
        self.swing_imu = Imu()
        # 
        self.prev_boom_imu = Imu()
        self.prev_arm_imu = Imu()
        self.prev_bucket_imu = Imu()
        self.prev_swing_imu = Imu()
        # self.prev_boom_ang = 0
        # self.prev_arm_ang = 0
        # self.prev_bucket_ang = 0
        # self.prev_swing_ang = 0

        self.joint_status = JointState()
        self.joint_status.header.frame_id = "base_link"
        self.joint_status.header.stamp = rospy.Time.now()
        self.joint_status.name = ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
        self.joint_status.position = [0, 0, 0, 0]
        self.joint_status.velocity = [0, 0, 0, 0]
        self.joint_status.effort = [0, 0, 0, 0]
        
        self.filtered_boom_imu_pose = 0
        self.filtered_arm_imu_pose = 0
        self.filtered_bucket_imu_pose = 0

        self.prev_swing_ang = 0
        self.prev_boom_ang = 0
        self.prev_arm_ang = 0
        self.prev_bucket_ang = 0

        self.prev_swing_imu_t = rospy.Time.now()
        self.prev_boom_imu_t = rospy.Time.now()
        self.prev_arm_imu_t = rospy.Time.now()
        self.prev_bucket_imu_t = rospy.Time.now()
        self.swing_imu_update = False
        self.boom_imu_update = False
        self.arm_imu_update = False
        self.bucket_imu_update = False

        # self.ave_swing_ang = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        # self.ave_boom_ang = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        # self.ave_arm_ang = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        # self.ave_bucket_ang = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        rospy.sleep(1.0)
        self.cnt = 0
        # start_time = rospy.Time.now()

        rospy.Timer(rospy.Duration(1.0/self.hz), self.timerCallback)

    def timerCallback(self, event):
        if self.cnt < 10:
            # self.filtered_swing_imu_pose = self.get_pose(self.boom_imu, self.swing_imu_rot_axis)
            self.filtered_boom_imu_pose = self.get_pose(self.boom_imu, self.boom_imu_rot_axis)
            self.filtered_arm_imu_pose = self.get_pose(self.arm_imu, self.arm_imu_rot_axis)
            self.filtered_bucket_imu_pose = self.get_pose(self.bucket_imu, self.bucket_imu_rot_axis)
            self.cnt += 1
            return
        if not (self.swing_imu_update and self.boom_imu_update and self.arm_imu_update and self.bucket_imu_update):
            return
        # boom_ang = self.get_joint_angle(self.swing_imu, self.boom_imu)
        # arm_ang = self.get_joint_angle(self.boom_imu, self.arm_imu)
        # bucket_ang = self.get_joint_angle(self.arm_imu, self.bucket_imu)
        # boom_rpy = self.eular_angle_from_quaternion(self.boom_imu.orientation)
        # arm_rpy = self.eular_angle_from_quaternion(self.arm_imu.orientation)
        # bucket_rpy = self.eular_angle_from_quaternion(self.bucket_imu.orientation)
        # print("boom   : {:.4f}, {:.4f}, {:.4f}".format(np.rad2deg(boom_rpy[0]), np.rad2deg(boom_rpy[1]), np.rad2deg(boom_rpy[2])))
        # print("arm    : {:.4f}, {:.4f}, {:.4f}".format(np.rad2deg(arm_rpy[0]), np.rad2deg(arm_rpy[1]), np.rad2deg(arm_rpy[2])))
        # print("bucket : {:.4f}, {:.4f}, {:.4f}".format(np.rad2deg(bucket_rpy[0]), np.rad2deg(bucket_rpy[1]), np.rad2deg(bucket_rpy[2])))
        # data = np.rad2deg(self.get_pose(self.boom_imu, "z"))
        # print("boom   : {:.4f}".format(np.rad2deg(self.get_pose(self.boom_imu, "z"))))
        # print("arm    : {:.4f}".format(np.rad2deg(self.get_pose(self.arm_imu, "z"))))
        # print("bucket : {:.4f}".format(np.rad2deg(self.get_pose(self.bucket_imu, "z"))))
        ###########################
        # swing_accel_pose_raw = self.get_pose(self.boom_imu, self.swing_imu_rot_axis)
        boom_accel_pose_raw = self.get_pose(self.boom_imu, self.boom_imu_rot_axis)
        arm_accel_pose_raw = self.get_pose(self.arm_imu, self.arm_imu_rot_axis)
        bucket_accel_pose_raw = self.get_pose(self.bucket_imu, self.bucket_imu_rot_axis)
        # self.filtered_swing_imu_pose = self.get_pose(self.boom_imu, self.swing_imu_rot_axis)
        self.filtered_boom_imu_pose = self.complementary_filter(self.filtered_boom_imu_pose, boom_accel_pose_raw, self.boom_imu.angular_velocity.z)
        self.filtered_arm_imu_pose = self.complementary_filter(self.filtered_arm_imu_pose, arm_accel_pose_raw, self.arm_imu.angular_velocity.z)
        self.filtered_bucket_imu_pose = self.complementary_filter(self.filtered_bucket_imu_pose, bucket_accel_pose_raw, self.bucket_imu.angular_velocity.z)
        # self.filtered_boom_imu_pose = self.normalize_PI(self.filtered_boom_imu_pose)
        # self.filtered_arm_imu_pose = self.normalize_PI(self.filtered_arm_imu_pose)
        # self.filtered_bucket_imu_pose = self.normalize_PI(self.filtered_bucket_imu_pose)
        swing_ang = 0 
        boom_ang = self.filtered_boom_imu_pose - self.init_boom_ang_offset
        arm_ang = self.filtered_arm_imu_pose - self.filtered_boom_imu_pose - self.init_arm_ang_offset
        bucket_ang = self.filtered_bucket_imu_pose - self.filtered_arm_imu_pose - self.init_bucket_ang_offset
        swing_ang = self.normalize_PI(swing_ang)
        boom_ang = self.normalize_PI(boom_ang)
        arm_ang = self.normalize_PI(arm_ang)
        bucket_ang = self.normalize_PI(bucket_ang)
        # print("boom   : {:.4f}".format(self.filtered_boom_imu_pose))
        # print("arm    : {:.4f}".format(self.filtered_arm_imu_pose))
        # print("bucket : {:.4f}".format(self.filtered_bucket_imu_pose))
        # self.ave_swing_ang = np.append(self.ave_swing_ang, swing_ang)
        # self.ave_boom_ang = np.append(self.ave_boom_ang, boom_ang)
        # self.ave_arm_ang = np.append(self.ave_arm_ang, arm_ang)
        # self.ave_bucket_ang = np.append(self.ave_bucket_ang, bucket_ang)
        # # self.ave_boom_ang.append(boom_ang)
        # self.ave_arm_ang.append(arm_ang)
        # self.ave_bucket_ang.append(bucket_ang)
        # self.ave_swing_ang.pop()
        # self.ave_boom_ang.pop()
        # self.ave_arm_ang.pop()
        # self.ave_bucket_ang.pop()
        # self.ave_swing_ang = np.delete(self.ave_swing_ang, 0)
        # self.ave_boom_ang = np.delete(self.ave_boom_ang, 0)
        # self.ave_arm_ang = np.delete(self.ave_arm_ang, 0)
        # self.ave_bucket_ang = np.delete(self.ave_bucket_ang, 0)
        # print("boom   : {:.4f}".format(np.rad2deg(boom_ang)))
        # print("arm    : {:.4f}".format(np.rad2deg(arm_ang)))
        # print("bucket : {:.4f}".format(np.rad2deg(bucket_ang)))
        # swing_ang = np.mean(self.ave_swing_ang)
        # boom_ang = np.mean(self.ave_boom_ang)
        # arm_ang = np.mean(self.ave_arm_ang)
        # bucket_ang = np.mean(self.ave_bucket_ang)
        print("boom   : {:.4f}".format(np.rad2deg(boom_ang)))
        print("arm    : {:.4f}".format(np.rad2deg(arm_ang)))
        print("bucket : {:.4f}".format(np.rad2deg(bucket_ang)))
        # print(self.ave_boom_ang)
        # print(self.ave_arm_ang)
        # print(self.ave_bucket_ang)
        self.joint_status.header.stamp = rospy.Time.now()
        self.joint_status.position[0] = 0 # swing_ang
        self.joint_status.position[1] = boom_ang
        self.joint_status.position[2] = arm_ang
        self.joint_status.position[3] = bucket_ang
        self.joint_status.velocity[0] = 0 # swing_ang
        self.joint_status.velocity[1] = (boom_ang - self.prev_boom_ang)/(self.boom_imu.header.stamp - self.prev_boom_imu_t).to_sec()
        self.joint_status.velocity[2] = (arm_ang - self.prev_arm_ang)/(self.arm_imu.header.stamp - self.prev_arm_imu_t).to_sec()
        self.joint_status.velocity[3] = (bucket_ang - self.prev_bucket_ang)/(self.bucket_imu.header.stamp - self.prev_bucket_imu_t).to_sec()
        self.pub_joint_state.publish(self.joint_status)

        self.prev_swing_imu_t = self.swing_imu.header.stamp
        self.prev_boom_imu_t = self.boom_imu.header.stamp
        self.prev_arm_imu_t = self.arm_imu.header.stamp
        self.prev_bucket_imu_t = self.bucket_imu.header.stamp
        self.prev_swing_ang = copy.deepcopy(swing_ang)
        self.prev_boom_ang = copy.deepcopy(boom_ang)
        self.prev_arm_ang = copy.deepcopy(arm_ang)
        self.prev_bucket_ang = copy.deepcopy(bucket_ang)
        self.swing_imu_update = False
        self.boom_imu_update = False
        self.arm_imu_update = False
        self.bucket_imu_update = False

    # def get_joint_angle(self, parent_imu, child_imu):
    #     parent_imu_matrix = tf_conversions.transformations.concatenate_matrices(
    #                             tf_conversions.transformations.translation_matrix([0, 0, 0]), 
    #                             tf_conversions.transformations.quaternion_matrix(
    #                                 [parent_imu.orientation.x, 1,#parent_imu.orientation.y,
    #                                  parent_imu.orientation.x, parent_imu.orientation.z])
    #                         )
    #     child_imu_matrix = tf_conversions.transformations.concatenate_matrices(
    #                             tf_conversions.transformations.translation_matrix([0, 0, 0]), 
    #                             tf_conversions.transformations.quaternion_matrix(
    #                                 [child_imu.orientation.x, child_imu.orientation.y,
    #                                  child_imu.orientation.x, child_imu.orientation.z])
    #                         )
    #     relative_tf = tf_conversions.transformations.inverse_matrix(parent_imu_matrix) * child_imu_matrix
    #     r, p, y = tf_conversions.transformations.euler_from_matrix(relative_tf)
    #     return [r, p, y]

    def get_pose(self, imu_data, axis):
        if axis == "x":
            return np.arctan2(imu_data.linear_acceleration.y, imu_data.linear_acceleration.z)
        if axis == "y":
            return np.arctan2(imu_data.linear_acceleration.z, imu_data.linear_acceleration.x)
        if axis == "z":
            return np.arctan2(imu_data.linear_acceleration.x, imu_data.linear_acceleration.y)

    def complementary_filter(self, data, theta_acc, theta_gyro):
        data_ = copy.deepcopy(data)
        data_ = data_ - self.omega*data_*(1.0/self.hz) + self.omega*theta_acc*(1.0/self.hz) + theta_gyro*(1.0/self.hz)
        # alpha = 0.01
        # data_ = (1 - alpha) * (data_ + theta_gyro*(1.0/self.hz)) + alpha * theta_acc
        return data_

    def cb_swing_imu(self, msg):
        self.swing_imu = msg
        self.swing_imu_update = True

    def cb_boom_imu(self, msg):
        self.boom_imu = msg
        self.boom_imu_update = True

    def cb_arm_imu(self, msg):
        self.arm_imu = msg
        self.arm_imu_update = True

    def cb_bucket_imu(self, msg):
        self.bucket_imu = msg
        self.bucket_imu_update = True

    # def eular_angle_from_quaternion(self, orientation):
    #     e_angle = tf_conversions.transformations.euler_from_quaternion([
    #                 orientation.x,
    #                 orientation.y,
    #                 orientation.z,
    #                 orientation.w
    #             ])
    #     return e_angle

    def normalize_PI(self, theta):
        while abs(theta) > np.pi:
            if theta > np.pi:
                theta -= np.pi*2.0
            elif theta <= -np.pi:
                theta += np.pi*2.0
        return theta

if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            backhoe_joint = BackhoeJointStatus()
            rospy.spin()
        except rospy.ROSInterruptException: pass
