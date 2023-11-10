#!/usr/bin/env python3

__author__ = "Lucas Barros"

import rospy

from esc_node import EscControl
from kill_switch import Kill

if __name__ == "__main__":
    rospy.init_node('hover')

    setpoint = EscControl()
    k = Kill(pin=4)
    k.listen()

    rospy.spin()