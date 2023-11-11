import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class Camera(object):

    def __init__(self)-> None:
        # Abra a câmera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("camera n abriu")
        
        # Crie um objeto CvBridge
        self.bridge = CvBridge()
        
        # Crie o publicador
        self.pub = rospy.Publisher('camera/image', Image, queue_size=10)

    def publish_image(self)-> None:
        ret, frame = self.cap.read()
        
        # Verifique se o frame foi lido corretamente
        if not ret:
            print("Erro ao ler o frame da câmera")
            return

        # Converta a imagem do OpenCV para uma mensagem ROS
        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        # Publique a imagem
        self.pub.publish(ros_image)

    def close_camera(self)-> None:
        # Feche a câmera quando terminar
        self.cap.release()

if __name__ == "__main__":
    rospy.init_node('image_publisher', anonymous=True)
    
    camera = Camera()
    
    while not rospy.is_shutdown():
        camera.publish_image()
        
    camera.close_camera()