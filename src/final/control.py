#!/usr/bin/env python3

__author__ = "Lucas Barros"

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class Control(object):

    # Construtor da classe Hover
    def __init__(self)-> None:

        # "Escuta" (o tópico /camera/orange) as imagens da câmera e chama a função image_callback() para processá-las
        self.control_sub = rospy.Subscriber('/control', Int32, self.controller) #NÃO RECEBE IMAGE

        # Publica no tópico /cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
        
        # Cria um objeto twist do tipo Twist para definir a velocidade do robô
        self.twist = Twist()
        
        # Definindo variáveis para auxiliar no controle
        self.error_list = []
        
        #self.prev_error = 0.0
        # self.int_error = 0.0  
         
        
    # Método responsável pelo controle PID do giro do Hovercraft
    def controller(self, msg:Int32)-> None:
        
        error = msg.data

        # Constantes de controle PID
        kp = 1 
        ki = 0 # .00000001
        kd = 0 #.000005
          
        # Lista de erros (evitando overshoot)     
        self.error_list.append(error)
        if(len(self.error_list) > 10):
            self.error_list.pop(0)
            
        # Média dos 10 últimos erros
        mean_error = sum(self.error_list) / len(self.error_list)

        # Cálculo da integral do erro
        #self.int_error = self.int_error + error

        # Equação do controle PID
        spin = kp * mean_error + kd * mean_error # mudar depois para kp * error

        self.twist.angular.z = spin
        self.twist.linear.x = 0.8
        
        # if error == 0:
        #     self.off = True
                            
        # if self.off:
        #     self.twist.angular.z = 0
        #     self.twist.linear.x = 0
        
        self.cmd_vel_pub.publish(self.twist)
        
        rospy.loginfo('velocidade angular:' + str(self.twist.angular.z))
      