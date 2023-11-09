#!/usr/bin/env python3

__author__ = "Lucas Barros"

import rospy

from esc_node import EscControl

if __name__ == "__main__":
    rospy.init_node('hover')

    setpoint = EscControl()

    rospy.spin()