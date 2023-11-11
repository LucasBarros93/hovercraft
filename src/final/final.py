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
    
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        camera.publish_image()
        rate.sleep()
    
    camera.close_camera()
