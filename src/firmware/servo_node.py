#!/usr/bin/env python3

__author__ = "Lucas Barros"

import rospy, pigpio, time
from geometry_msgs.msg import Twist

def _map(value:float, from_low:float, from_high:float, to_low:float, to_high:float)-> float:
    # Mapeia o valor de from_low/from_high para to_low/to_high
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

class Servo:
    MIN_WIDTH = 600 #menor angulo em teoria
    MAX_WIDTH = 2400 #maior angulo em teoria, cuidado ppra n quebrar essa porra
     
    def __init__(self, pin:int)-> None:
        self.pin = pin
        self.conn = pigpio.pi()
        
    def test(self) -> None:
        self.conn.set_servo_pulsewidth(self.pin ,1500) # centro
        time.sleep(3)
        
        self.conn.set_servo_pulsewidth(self.pin ,1000) # um pouco anti horario
        time.sleep(3)
        
        self.conn.set_servo_pulsewidth(self.pin ,2000) # um pouco horario
        time.sleep(3)
        
    def control(self, pos:int)-> None:
        if pos < self.MAX_WIDTH and pos > self.MIN_WIDTH:
            self.conn.set_servo_pulsewidth(self.pin, pos)

class ServoControl(object):

    # Construtor da classe Hover
    def __init__(self)-> None:

        self.control_sub = rospy.Subscriber('/cmd_vel', Twist, self.servo_control)
        
    def servo_control(self, msg:Twist):
        pos = msg.angular.z
        servo = Servo(pin=18)
        
        pos = int(_map(pos, -1, 1, 600, 2400))
        
        servo.control(pos=pos)
        
        