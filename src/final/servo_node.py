#!/usr/bin/env python3

__author__ = "Lucas Barros"

import rospy, pigpio, time
from geometry_msgs.msg import Twist
from motors import Servo, ESC

def _map(value:float, from_low:float, from_high:float, to_low:float, to_high:float)-> float:
    # Mapeia o valor de from_low/from_high para to_low/to_high
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low


class ServoControl(object):

    # Construtor da classe Hover
    def __init__(self)-> None:

        self.control_sub = rospy.Subscriber('/cmd_vel', Twist, self.servo_control)
        self.servo = Servo(pin=18)
        #self.esc = ESC(pin1= 13)
        
    def servo_control(self, msg:Twist):
        
        pos = msg.angular.z
        pos = int(_map(pos, -1, 1, 1000, 2000))
        
        # if msg.linear.x != 0:
        #     self.esc.arm()
        # else:
        #     self.esc.halt()
        
        self.servo.control(pos=pos)
        
        rospy.loginfo('pos sero:' + str(pos))
        
        