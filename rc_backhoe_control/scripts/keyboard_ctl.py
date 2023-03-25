#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int8MultiArray

def talker():
    pub = rospy.Publisher('cmd/actuator', Int8MultiArray, queue_size=10)
    rospy.init_node('keyboard_ctl', anonymous=True)
    cmd = Int8MultiArray(data=[0,0,0,0])
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        key  = input("Input boom↑↓f, b, l, r, stop q : ") # 標準入力からキーを読み込む
        print(key)     # 読み込んだキーの値を標準出力へ出力

        if key == 'f': # fキーが押されていたら
            cmd.data[0]  =  1
        elif key == 'b':
            cmd.data[0]  =  -1
        elif key == 'l':
            cmd.data[1]  =  1
        elif key == 'r':
            cmd.data[1]  =  -1
        elif key == 'd':
            cmd.data[2]  =  1
        elif key == 'z':
            cmd.data[2]  =  -1
        elif key == 'y':
            cmd.data[3]  =  1
        elif key == 'h':
            cmd.data[3]  =  -1
        elif key == 'q':
            cmd.data = [0,0,0,0]
        else:
            print("Input f, b, l, r, stop q")
        
        pub.publish(cmd)    # 速度指令メッセージをパブリッシュ
        rate.sleep()        # 指定した周期でループするよう寝て待つ


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass