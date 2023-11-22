#!/usr/bin/env python3

# python
import rospy

# std_msgs/String 타입 메시지 import
from std_msgs.msg import String

def talker():
    #talker node 생성
    rospy.init_node('talker', anonymous=True)
    #talker 노드는 String 타입의 chatter 토픽 발행
    pub = rospy.Publisher('chatter', String, queue_size=10)

    # 1초에 10번 추가 rate 생성
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        #hello_str 토픽 발행
        pub.publish(hello_str)
        #rate 주기로 발행
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: #shutdown 이전까지 실행시켜준다
        pass
