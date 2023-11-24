#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import CtrlCmd


class s_drive:
    def __init__(self):
        rospy.init_node("s_drive", anonymous=True)
        # /ctrl_cmd 토픽 발행
        cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        rate = rospy.Rate(30)  # 30hz
        # CtrlCmd 메시지 생성
        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.velocity = 10
        steering_cmd = [-0.2, 0.2]
        cmd_cnts = 50  # 토픽 전송 수

        while not rospy.is_shutdown():
            for i in range(2):
                cmd.steering = steering_cmd[i]
                rospy.loginfo(cmd)
                for _ in range(cmd_cnts):
                    # 발행
                    cmd_pub.publish(cmd)
                    rate.sleep()


if __name__ == "__main__":
    try:
        s_d = s_drive()
    except rospy.ROSInterruptException:
        pass
