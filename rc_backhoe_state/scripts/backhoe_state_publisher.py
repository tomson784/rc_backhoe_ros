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

class BackhoeStatusPublisher():
    def __init__(self):
        rospy.init_node('backhoe_status')
        # param list
        self.hz = rospy.get_param('~hz', 1.0)

        # publisher list
        self.pub_software_on = rospy.Publisher('status/software_on', Bool, queue_size=10)
        # self.pub_joint_state = rospy.Publisher('status', JointState, queue_size=10)

        # subscriber list
        rospy.Subscriber("cmd/software_on", Bool, self.cb_cmd_software_on)
        self.software_on = False

        rospy.Timer(rospy.Duration(1.0/self.hz), self.timerCallback)

    def timerCallback(self, event):
        self.pub_software_on.publish(self.software_on)
        # print(self.software_on)

    def cb_cmd_software_on(self, msg):
        self.software_on = msg.data

if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            backhoe_status = BackhoeStatusPublisher()
            rospy.spin()
        except rospy.ROSInterruptException: pass
