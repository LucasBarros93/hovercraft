import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def camera_calibration(img):

    camera_matrix = np.array([[314.000745, 0.000000, 309.555461], [0.000000, 315.533321, 223.927141],[0.000000, 0.000000, 1.000000]])
    distorcion_coef = np.array([[-0.280652, 0.054617, -0.000586, -0.002693, 0.000000]])

    w = img.shape[0]
    h = img.shape[1]

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distorcion_coef, (w, h), 1, (w,h))
    img = cv2.undistort(img, camera_matrix, distorcion_coef, None, newcameramtx)

    x, y, w, h = roi

    return img

def capture_and_publish_image():
    # Inicialize o nó ROS
    rospy.init_node('image_publisher', anonymous=True)

    # Crie um objeto CvBridge
    bridge = CvBridge()

    # Abra a câmera
    cap = cv2.VideoCapture(0)

    # Verifique se a câmera foi aberta corretamente
    if not cap.isOpened():
        raise IOError("Não foi possível abrir a câmera")

    # Crie o publicador
    pub = rospy.Publisher('camera/image', Image, queue_size=10)

    while not rospy.is_shutdown():
        # Capture a imagem da câmera
        ret, frame = cap.read()

        # Verifique se a imagem foi capturada corretamente
        if not ret:
            break

        # Converta a imagem do OpenCV para uma mensagem ROS
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")

        # Publique a imagem
        pub.publish(ros_image)

    # Feche a câmera quando terminar
    cap.release()

if __name__ == '__main__':
    capture_and_publish_image()

