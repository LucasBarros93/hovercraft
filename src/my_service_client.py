#!/usr/bin/env python

import rospy
from project_ws.srv import MyService, MyServiceRequest

def my_service_client(enable_twist):
    rospy.wait_for_service('my_service')
    try:
        my_service = rospy.ServiceProxy('my_service', MyService)
        response = my_service(enable_twist)
        return response
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('my_service_client')
    enable_twist = MyServiceRequest()
    enable_twist.enable = True  # Altere para True ou False, conforme necessário
    response = my_service_client(enable_twist)
    if response is not None:
        print(f"Received Twist message: {response.twist}")
