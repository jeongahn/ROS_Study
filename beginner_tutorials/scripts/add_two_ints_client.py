#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from beginner_tutorials.srv import AddTwoInts

def add_two_ints_client(x,y):
    # add_two_ints 라는 이름의 서비스를 사용할 수 있을 때까지 대기
    rospy.wait_for_service('add_two_ints')
    try:
        # 서비스 요청 핸들러
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        # 서비스 요청 전송 후 응답 대기
        resp1 = add_two_ints(x,y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])

    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x,y))
    print("%s + %s = %s"%(x,y,add_two_ints_client(x,y)))
