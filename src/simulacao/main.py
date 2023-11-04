#!/usr/bin/env python3

__author__ = "Lucas Barros"

import rospy, cv2, cv_bridge
import numpy as np

from control import Control
from setpoint import SetPoint

if __name__ == "__main__":
    rospy.init_node('hover')

    setpoint = SetPoint()
    control = Control()

    rospy.spin()