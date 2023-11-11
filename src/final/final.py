#!/usr/bin/env python3

__author__ = "Lucas Barros"

import rospy

from camera import Camera
from control import Control
from setpoint import SetPoint

from servo_node import ServoControl

if __name__ == "__main__":
    
    rospy.init_node('hover', anonymous=True)

    setpoint = SetPoint()
    control = Control()
    camera = Camera()
    servo = ServoControl()
    
    while not rospy.is_shutdown():
        camera.publish_image()
    
    camera.close_camera()
