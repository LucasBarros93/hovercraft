import cv2
import numpy as np
from pubcam import camera_calibration  
# Função para capturar e exibir o vídeo da câmera

def nothing(x):
    pass
cv2.namedWindow('Image')
cv2.createTrackbar('Hue1', 'Image', 0, 179, nothing)
cv2.createTrackbar('Saturation1', 'Image', 0, 255, nothing)
cv2.createTrackbar('Value1', 'Image', 0, 255, nothing)
cv2.createTrackbar('Hue', 'Image', 0, 179, nothing)
cv2.createTrackbar('Saturation', 'Image', 0, 255, nothing)
cv2.createTrackbar('Value', 'Image', 0, 255, nothing)

def mostrar_video_camera():
    # Abre a câmera
    cap = cv2.VideoCapture(0)

    # Verifica se a câmera foi aberta com sucesso
    if not cap.isOpened():
        print("Erro ao abrir a câmera")
        return

    # Loop para capturar e exibir o vídeo
    while True:
        # Captura o frame da câmera
        ret, frame = cap.read()
        #frame = camera_calibration(frame)

        # Verifica se a captura foi bem-sucedida
        if not ret:
            print("Erro ao capturar o frame")
            break
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h1 = cv2.getTrackbarPos('Hue1', 'Image')
        h = cv2.getTrackbarPos('Hue', 'Image')
        s1 = cv2.getTrackbarPos('Saturation1', 'Image')
        s = cv2.getTrackbarPos('Saturation', 'Image')
        v1 = cv2.getTrackbarPos('Value1', 'Image')
        v = cv2.getTrackbarPos('Value', 'Image')

        lower_green = np.array([h, s, v])
        upper_green = np.array([h1, s1, v1])

        mask = cv2.inRange(hsv, lower_green, upper_green)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Exibe o frame
        cv2.imshow('Video da Camera', frame)
        cv2.imshow('Resultado do filtro', result)
        # Verifica se a tecla 'q' foi pressionada para encerrar o loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libera os recursos ao encerrar
    cap.release()
    cv2.destroyAllWindows()

# Chama a função para mostrar o vídeo da câmera
mostrar_video_camera()
