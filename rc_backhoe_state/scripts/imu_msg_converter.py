#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu

pub_boom_imu_msgs = rospy.Publisher('boom/imu/data_raw', Imu, queue_size=1)
pub_arm_imu_msgs = rospy.Publisher('arm/imu/data_raw', Imu, queue_size=1)
pub_bucket_imu_msgs = rospy.Publisher('bucket/imu/data_raw', Imu, queue_size=1)
pub_swing_imu_msgs = rospy.Publisher('swing/imu/data_raw', Imu, queue_size=1)

boom_imu_raw = Imu()
arm_imu_raw = Imu()
bucket_imu_raw = Imu()
swing_imu_raw = Imu()

def cb_boom_data(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    boom_imu_raw.header.frame_id = "map"
    boom_imu_raw.header.stamp = rospy.Time.now()
    boom_imu_raw.linear_acceleration.x = msg.data[0] / 16384.0 * 9.806
    boom_imu_raw.linear_acceleration.y = msg.data[1] / 16384.0 * 9.806
    boom_imu_raw.linear_acceleration.z = msg.data[2] / 16384.0 * 9.806
    boom_imu_raw.angular_velocity.x = msg.data[3] / 131.0 * 0.0174533
    boom_imu_raw.angular_velocity.y = msg.data[4] / 131.0 * 0.0174533
    boom_imu_raw.angular_velocity.z = msg.data[5] / 131.0 * 0.0174533
    pub_boom_imu_msgs.publish(boom_imu_raw)
    
def cb_arm_data(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    arm_imu_raw.header.frame_id = "map"
    arm_imu_raw.header.stamp = rospy.Time.now()
    arm_imu_raw.linear_acceleration.x = msg.data[0] / 16384.0 * 9.806
    arm_imu_raw.linear_acceleration.y = msg.data[1] / 16384.0 * 9.806
    arm_imu_raw.linear_acceleration.z = msg.data[2] / 16384.0 * 9.806
    arm_imu_raw.angular_velocity.x = msg.data[3] / 131.0 * 0.0174533
    arm_imu_raw.angular_velocity.y = msg.data[4] / 131.0 * 0.0174533
    arm_imu_raw.angular_velocity.z = msg.data[5] / 131.0 * 0.0174533
    pub_arm_imu_msgs.publish(arm_imu_raw)

def cb_bucket_data(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    bucket_imu_raw.header.frame_id = "map"
    bucket_imu_raw.header.stamp = rospy.Time.now()
    bucket_imu_raw.linear_acceleration.x = msg.data[0] / 16384.0 * 9.806
    bucket_imu_raw.linear_acceleration.y = msg.data[1] / 16384.0 * 9.806
    bucket_imu_raw.linear_acceleration.z = msg.data[2] / 16384.0 * 9.806
    bucket_imu_raw.angular_velocity.x = msg.data[3] / 131.0 * 0.0174533
    bucket_imu_raw.angular_velocity.y = msg.data[4] / 131.0 * 0.0174533
    bucket_imu_raw.angular_velocity.z = msg.data[5] / 131.0 * 0.0174533
    pub_bucket_imu_msgs.publish(bucket_imu_raw)

def cb_swing_data(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    swing_imu_raw.header.frame_id = "map"
    swing_imu_raw.header.stamp = rospy.Time.now()
    swing_imu_raw.linear_acceleration.x = msg.data[0] / 16384.0 * 9.806
    swing_imu_raw.linear_acceleration.y = msg.data[1] / 16384.0 * 9.806
    swing_imu_raw.linear_acceleration.z = msg.data[2] / 16384.0 * 9.806
    swing_imu_raw.angular_velocity.x = msg.data[3] / 131.0 * 0.0174533
    swing_imu_raw.angular_velocity.y = msg.data[4] / 131.0 * 0.0174533
    swing_imu_raw.angular_velocity.z = msg.data[5] / 131.0 * 0.0174533
    pub_swing_imu_msgs.publish(swing_imu_raw)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("boom/imu/data_raw_bit", Int32MultiArray, cb_boom_data)
    rospy.Subscriber("arm/imu/data_raw_bit", Int32MultiArray, cb_arm_data)
    rospy.Subscriber("bucket/imu/data_raw_bit", Int32MultiArray, cb_bucket_data)
    rospy.Subscriber("swing/imu/data_raw_bit", Int32MultiArray, cb_swing_data)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()