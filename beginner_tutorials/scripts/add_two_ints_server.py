#!/usr/bin/env python3

from __future__ import print_function
# 서비스 파일 import
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse
import rospy

# respose 함수 생성
def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    # handle_add_two_ints가 response 함수인 add_two_ints 서비스 생성
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()