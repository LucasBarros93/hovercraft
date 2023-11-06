#!/usr/bin/env python3

__author__ = "Lucas Barros"

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32

def _map(value:float, from_low:float, from_high:float, to_low:float, to_high:float)-> float:
    # Mapeia o valor de from_low/from_high para to_low/to_high
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low


class SetPoint(object): #qual é do 'object'?

    # Construtor da classe Hover
    def __init__(self)-> None:

        # Atalho converter a mensagem Image em imagem para OpenCV
        self.bridge = cv_bridge.CvBridge()

        # "Escuta" (o tópico /camera/orange) as imagens da câmera e chama a função image_callback() para processá-las
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback) 

        # Publica no tópico /control
        self.cmd_control__pub = rospy.Publisher('/control', Int32, queue_size=1)

        # Definindo as cores desejadas
        self.green = np.array([[35, 100, 100], [85, 255, 255]])  #HSV
        self.red = np.array([[0, 100, 100], [10, 255, 255]]) #HSV

    def image_callback(self, msg:Image)-> None:
        
        # Converte a mensagem em numpy array
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Cria as máscaras que enxergarão verde e vermelho
        green_mask = cv2.inRange(hsv, self.green[0], self.green[1])
        red_mask = cv2.inRange(hsv, self.red[0], self.red[1])



        
        # Criando a máscara que terá os dois maiores contornos para o verde
        green_mask_largest = np.zeros_like(green_mask)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(green_contours) >= 2:

            # Classificando os contornos pelo tamanho em ordem crescente
            green_contours = sorted(green_contours, key = cv2.contourArea, reverse = True)

            # Pegando os dois maiores contornos
            green_largest_contours = green_contours[:2] #TINHA UM 2 AI, SÓ MUUDEI PRA 1, ESPERO Q DE BOM

            # Desenhando os dois maiores contornos na máscara
            for contour in green_largest_contours:
                cv2.drawContours(green_mask_largest, [contour], 0, 255, -1)

        # Criando a máscara que terá os dois maiores contornos para o vermelho
        red_mask_largest = np.zeros_like(red_mask)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(red_contours) >= 2:

            # Classificando os contornos pelo tamanho em ordem crescente
            red_contours = sorted(red_contours, key = cv2.contourArea, reverse = True)

            # Pegando os dois maiores contornos
            red_largest_contours = red_contours[:2] #TINHA UM 2 AI, SÓ MUUDEI PRA 1, ESPERO Q DE BOM

            # Desenhando os dois maiores contornos na máscara
            for contour in red_largest_contours:
                cv2.drawContours(red_mask_largest, [contour], 0, 255, -1)




        # Dimensões da tela
        height, width, depth = image.shape

        # Centro de massa dos pixels
        gM = cv2.moments(green_mask_largest)
        rM = cv2.moments(red_mask_largest)

        k = 0.1  #constante para ajustar a escala do erro
        
        error = (gM['m00'] - rM['m00']) * k
        
        mg = Int32()
        mg.data = int(error)
        
        self.cmd_control__pub.publish(mg)
            
        try:
            # Desenhe o círculo na imagem
            point = width/2 + _map(error, -40000, 40000, -320, 320)
            cv2.circle(image, (int(np.ceil(point)), int(np.ceil(height/2))), 10, (0, 0, 255), -1)  # O valor -1 preenche o círculo
        except Exception as e:
            print("no mass center found\n", e)

        # Mostra a imagem vista pelo hover
        cv2.imshow("Hover\'s Vision", green_largest_contours)
        # cv2.imshow("MASK", mask)
        cv2.waitKey(3)
         
        #xCM_red ~ 640
        #xCM_green ~ 200
        #print(xCM)