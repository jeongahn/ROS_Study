#!/usr/bin/env python3

# python ROS library import
import rospy 
# std_msgs.msg import String
from std_msgs.msg import String

def callback(data):
    # data 정보 출력
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    #listener 노드 생성
    rospy.init_node('listener', anonymous=True)
    #chatter 토픽 구독
    rospy.Subscriber("chatter", String, callback)
    # 노드가 멈출 때까지 대기
    rospy.spin()

if __name__ == '__main__':
    listener()