#!/usr/bin/env python3

__author__ = "Marco A G Garcia"

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

class Hover:

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
        self.lower_orange = np.array([5, 100, 100])  #HSV
        self.upper_orange = np.array([15, 255, 255]) #HSV

    # Método que processa as imagens recebidas e toma a decisão
    def image_callback(self, msg):

        # Velocidade padrão do Hovercraft
        self.twist.linear.x = 0.2
        self.cmd_vel_pub.publish(self.twist)
        
        # Converte a mensagem em numpy array
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Cria a máscara que enxerga laranja
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        # Dimensões da tela
        height, width, depth = image.shape

        # Contabiliza o momento dos pixels laranjas da imagem
        M = cv2.moments(mask)

        # Se existe laranja na câmera:
        if M['m00'] > 0:

            # M['m10'] é o momento horizontal dos pixels
            mx = int(M['m10']/M['m00'])

            # CONTROLE
            error = mx - width/2
            self.twist.angular.z = -float(error) / 1000
            self.cmd_vel_pub.publish(self.twist)
            image = cv2.putText(image, str(error), (0,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (50,150,255), 3, cv2.LINE_AA)
            
        # Mostra a imagem vista pelo hover
        cv2.imshow("Hover\'s Vision", image)
        # cv2.imshow("MASK", mask)
        cv2.waitKey(3)
    
rospy.init_node('hover')
hover = Hover()
rospy.spin()

# END ALL
