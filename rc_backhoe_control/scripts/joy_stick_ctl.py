#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Joy

cmd = Int8MultiArray(data=[0,0,0,0])

def cb_joy(msg):
    global cmd
    # swing left, right
    if msg.axes[3] > 0.7:
        cmd.data[0] = 1
    elif msg.axes[3] < -0.7:
        cmd.data[0] = -1
    else:
        cmd.data[0] = 0
    # boom up, down
    if msg.axes[1] > 0.7:
        cmd.data[1] = 1
    elif msg.axes[1] < -0.7:
        cmd.data[1] = -1
    else:
        cmd.data[1] = 0
    # arm up, down
    if msg.axes[4] > 0.7:
        cmd.data[2] = 1
    elif msg.axes[4] < -0.7:
        cmd.data[2] = -1
    else:
        cmd.data[2] = 0
    # bucket up, down
    if msg.axes[5] < -0.7:
        cmd.data[3] = 1
    elif msg.axes[2] < -0.7:
        cmd.data[3] = -1
    else:
        cmd.data[3] = 0



def talker():
    global cmd
    pub = rospy.Publisher('cmd/actuator', Int8MultiArray, queue_size=10)
    rospy.init_node('keyboard_ctl', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        pub.publish(cmd)    # 速度指令メッセージをパブリッシュ
        # cmd.data = [0,0,0,0]
        rate.sleep()        # 指定した周期でループするよう寝て待つ


if __name__ == '__main__':
    rospy.Subscriber("joy", Joy, cb_joy)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass