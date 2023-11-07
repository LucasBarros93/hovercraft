#!/usr/bin/env python

import rospy
from project_ws.srv import MyService, MyServiceResponse
from geometry_msgs.msg import Twist

def handle_my_service(req):
    if req.enable:
        # Crie uma mensagem Twist
        twist_msg = Twist()

        # Personalize a mensagem Twist de acordo com sua necessidade
        twist_msg.linear.x = 0.2
        rospy.init_node('hover', anonymous=True)

        # Publique a mensagem Twist no tópico desejado
        twist_pub.publish(twist_msg)
        rospy.loginfo("Twist message published.")
        return MyServiceResponse(twist_msg)
    
    else:
        # Lógica para desativar o Twist, se necessário
        twist_msg.linear.x = 0.0
        return MyServiceResponse(None)

def my_service_server():
    rospy.init_node('my_service_server')
    s = rospy.Service('my_service', MyService, handle_my_service)
    print("My Service is ready.")
    rospy.spin()

if __name__ == "__main__":
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    my_service_server()
