#!/usr/bin/env python3

__author__ = "Lucas Barros"

import rospy, pigpio, time
from geometry_msgs.msg import Twist
from motors import Servo, ESC

from std_srvs.srv import Empty, EmptyResponse

def _map(value:float, from_low:float, from_high:float, to_low:float, to_high:float)-> float:
    # Mapeia o valor de from_low/from_high para to_low/to_high
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low


class ServoControl(object):

    # Construtor da classe Servo
    def __init__(self)-> None:

        self.control_sub = rospy.Subscriber('/cmd_vel', Twist, self.servo_control)
        self.servo = Servo(pin=18)
        self.esc = ESC(pin1= 13)
        
        # Service pra parar o bicho (e pra andar tbm?)
        self.on_srv = rospy.Service("/turn_on", Empty, self.on_off)
        self.off = True
        
    def servo_control(self, msg:Twist):
        
        if not self.off:
            pos = msg.angular.z
            pos = int(_map(pos, -1, 1, 1050, 1650)) # valores mínimo e máximo do servo
            
            self.servo.control(pos=pos)
            
            rospy.loginfo('pos servo:' + str(pos))
        
    
    def on_off(self, req:Empty)-> EmptyResponse:        
        self.off = not self.off
        
        if self.off:
            self.esc.halt()
        else:
            self.esc.pwm(1600)
        
        return EmptyResponse()        