#!/usr/bin/env python3

__author__ = "Lucas Barros"

import rospy, cv2, cv_bridge
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


class Control(object):

    # Construtor da classe Hover
    def __init__(self)-> None:

        # Atalho converter a mensagem Image em imagem para OpenCV
        self.bridge = cv_bridge.CvBridge()

        # "Escuta" (o tópico /camera/orange) as imagens da câmera e chama a função image_callback() para processá-las
        self.control_sub = rospy.Subscriber('/control', Int32, self.controller) #NÃO RECEBE IMAGE

        # Publica no tópico /cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 

        # Cria um objeto twist do tipo Twist para definir a velocidade do robô
        self.twist = Twist()
        
        # Definindo algumas variáveis para auxiliar no controle
        self.error_list = []
        
        self.prev_error = 0.0
        self.int_error = 0.0  
         
        
    # Método responsável pelo controle PID do giro do Hovercraft
    def controller(self, msg:Int32)-> None:
        
        last_error = msg.data

        # Constantes de controle PID
        kp = .00001 
        ki = .00000001
        kd = .00001
          
        #CALCULANDO MÉDIA DOS ULTIMOS Xcm     
        self.error_list.append(last_error)             #OLHA AQUI PPORRA  
        if(len(self.error_list) > 10):
            self.error_list.pop(0)
            
        error = sum(self.error_list) / len(self.error_list)

        # Cálculo da integral do erro
        self.int_error = self.int_error + error

        # Cálculo da derivada do erro
        der_error = error - self.prev_error
        self.prev_error = error

        # Equação do controle PID
        spin = kp * error + ki * self.int_error + kd * der_error

        self.twist.angular.z = spin
        if(error == 0):
            self.twist.angular.z = 0
            
        self.twist.linear.x = .4
        
        self.cmd_vel_pub.publish(self.twist)
        
        