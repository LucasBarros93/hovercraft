#!/usr/bin/env python3

__author__ = "Marco A G Garcia"

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

class Hover(object):

    # Construtor da classe Hover
    def __init__(self):

        # Atalho converter a mensagem Image em imagem para OpenCV
        self.bridge = cv_bridge.CvBridge()

        # "Escuta" (o tópico /camera/orange) as imagens da câmera e chama a função image_callback() para processá-las
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        # Publica no tópico /cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Cria um objeto twist do tipo Twist para definir a velocidade do robô
        self.twist = Twist()

        # Definindo as cores desejadas
        self.green = np.array([[35, 100, 100], [85, 255, 255]])  #HSV
        self.red = np.array([[0, 100, 100], [10, 255, 255]]) #HSV

        # Definindo algumas variáveis para auxiliar no controle
        self.prev_error = 0.0
        self.int_err = 0.0

    # Método que processa as imagens recebidas e toma a decisão
    def image_callback(self, msg):

        # Velocidade padrão do Hovercraft
        self.twist.linear.x = 0.1
        self.cmd_vel_pub.publish(self.twist)
        
        # Converte a mensagem em numpy array
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Cria as máscaras que enxergarão verde e vermelho
        green_mask = cv2.inRange(hsv, self.green[0], self.green[1])
        red_mask = cv2.inRange(hsv, self.red[0], self.red[1])

        # Dimensões da tela
        height, width, depth = image.shape

        # Ignorando o centro da máscara
        green_mask[0:height, int(1*width/3):int(2*width/3)] = 0 
        red_mask[0:height, int(1*width/3):int(2*width/3)] = 0 

        # Centro de massa dos pixels
        gM = cv2.moments(green_mask)
        rM = cv2.moments(red_mask)


        # Se existe verde ou vermelho na câmera:
        if gM['m00'] > 0 or rM['m00'] > 0:

            # M['m10'] é o momento horizontal dos pixels (m * x / m = x)
            xCM_green = int(gM['m10']/gM['m00'])
            xCM_red = int(rM['m10']/rM['m00'])

            # Chamando o método que calcula o giro do Hovercraft
            spin = self.controller(self, width, xCM_red - xCM_green)

            # Publicando o giro
            self.twist.angular.z = spin
            self.cmd_vel_pub.publish(self.twist)

        else:
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

        # Mostra a imagem vista pelo hover
        cv2.imshow("Hover\'s Vision", image)
        # cv2.imshow("MASK", mask)
        cv2.waitKey(3)

    # Método responsável pelo controle do giro do Hovercraft
    def controller(self, width, xCM):

        # Constantes de controle PID
        kp = 0.0
        ki = 0.0
        kd = 0.0

        # O centro da imagem é igual a largura/2. O vermelho estará na direita e o verde na esquerda sempre.
        # Portanto, o erro é a diferença do centro das latinhas - o centro da imagem 
        #           centro da img   centro das latinhas
    
        error = float(xCM - width/2.0)

        # Se xCM_red > xCM_green, erro  0. Portanto, deve virar a esquerda.
        # Se xCM_red < xCM_green, erro  0. Portanto, deve virar a direita.

        # Cálculo da integral do erro
        self.int_err = self.int_err + error

        # Cálculo da derivada do erro
        der_err = error - self.prev_err
        self.prev_err = error

        # Equação do controle PID
        spin = kp * error + ki * self.int_err + kd * der_err

        return spin

    
rospy.init_node('hover')
hover = Hover()
rospy.spin()

# END ALL