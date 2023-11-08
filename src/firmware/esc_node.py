#!/usr/bin/env python3

__author__ = "Lucas Barros"

from motors import ESC
import rospy
from std_srvs.srv import Empty, EmptyResponse

class EscControl(object):
    
    def __init__(self)-> None:
        self.serv_hover = rospy.Service('/hover', Empty, self.hover)
        
        self.esc = ESC(pin1=12)
        self.off = True
        
    def hover(self, req:Empty)-> EmptyResponse:
        self.off = not self.off
        
        if self.off:
            self.esc.halt()
        else:
            self.esc.arm()