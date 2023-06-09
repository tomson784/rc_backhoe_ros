#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def talker():
    rospy.init_node('joint_trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('/rc_backhoe_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(0.5)

    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = [ "swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
    msg.points = [JointTrajectoryPoint() for i in range(len(msg.joint_names))]
    msg.points[0].positions = [0.0, 0.0, 0.0, 0.0]
    msg.points[0].time_from_start = rospy.Time(1.0)
    msg.points[1].positions = [0.0, math.pi/6.0, math.pi/6.0, math.pi/6.0]
    msg.points[1].time_from_start = rospy.Time(2.0)
    msg.points[2].positions = [0.5, math.pi/6.0, math.pi/6.0, math.pi/6.0]
    msg.points[2].time_from_start = rospy.Time(3.0)
    msg.points[3].positions = [-0.5, math.pi/6.0, math.pi/6.0, math.pi/6.0]
    msg.points[3].time_from_start = rospy.Time(4.0)

    pub.publish(msg)
    rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass