#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def camera_calibration(img):

    camera_matrix = np.array([[334.074225, 0.0, 303.062046], [0.0, 334.213197, 241.575804], [0.0, 0.0, 1.0]])
    distorcion_coef = np.array([[-0.285926, 0.058121, 0.002099, 0.005293, 0.0]])

    w = img.shape[0]
    h = img.shape[1]

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distorcion_coef)
    img = cv2.undistort(img, camera_matrix, distorcion_coef, None, newcameramtx)

    x, y, w, h = roi
    img = img[y:y+h, x:x+w] 

    return img

def capture_and_publish_image():
    # Inicialize o n   ROS
    rospy.init_node('image_publisher', anonymous=True)

    # Crie um objeto CvBridge
    bridge = CvBridge()

    # Abra a c  mera
    cap = cv2.VideoCapture(2)

    # Verifique se a c  mera foi aberta corretamente
    if not cap.isOpened():
        raise IOError("N  o foi poss  vel abrir a c  mera")

    # Crie o publicador
    pub = rospy.Publisher('camera/image', Image, queue_size=10)

    while not rospy.is_shutdown():
        # Capture a imagem da c  mera
        ret, frame = cap.read()

        # Verifique se a imagem foi capturada corretamente
        if not ret:
            break


        # Converta a imagem do OpenCV para uma mensagem ROS
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")

        # Publique a imagem
        pub.publish(ros_image)

    # Feche a c  mera quando terminar
    cap.release()

if __name__ == '__main__':
    capture_and_publish_image()

