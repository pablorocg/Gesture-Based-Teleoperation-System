#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import ackermann_msgs.msg

# TODO Declarar el detector de pose de mediapipe a utilizar
mp_hands = mp.solutions.hands # Importar el modelo de detección de manos de MediaPipe
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5) # Inicializar el modelo de detección de manos
mp_drawing = mp.solutions.drawing_utils # Importar la librería de dibujo de MediaPipe

# Publicador de mensajes de control
ackermann_command_publisher = None

#Procesar la imagen del operador
def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convertir la imagen de ROS a una imagen de OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # TODO Procesar la imagen con MediaPipe
    image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) # Convertir la imagen a RGB
    results = hands.process(image_rgb) # Procesar la imagen con el modelo de MediaPipe

    if results.multi_hand_landmarks:# Si se detectaron landmarks en la mano
        for hand_landmarks in results.multi_hand_landmarks:# Por cada mano detectada
            # TODO Reconocer el gesto mediante alguna clasificación a partir de los landmarks
            
            # Obtener las coordenadas de las puntas de los dedos
            thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]# Coordenadas de la punta del pulgar
            index_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]# Coordenadas de la punta del índice
            middle_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]# Coordenadas de la punta del dedo medio
            ring_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]# Coordenadas de la punta del dedo anular
            pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]# Coordenadas de la punta del dedo meñique

            # Obtener las coordenadas de la base de la mano (para comparar la elevación de los dedos)
            wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]

            # Determinar qué dedo está levantado en comparación con la base de la mano (wrist)
            if thumb_tip.y < wrist.y and thumb_tip.y < index_finger_tip.y and \
                thumb_tip.y < middle_finger_tip.y and thumb_tip.y < ring_finger_tip.y and \
                thumb_tip.y < pinky_tip.y:

                command = "acelerar"

            elif index_finger_tip.y < wrist.y and index_finger_tip.y < thumb_tip.y and \
                index_finger_tip.y < middle_finger_tip.y and index_finger_tip.y < ring_finger_tip.y and \
                index_finger_tip.y < pinky_tip.y:

                command = "frenar"

            elif middle_finger_tip.y < wrist.y and middle_finger_tip.y < thumb_tip.y and \
                middle_finger_tip.y < index_finger_tip.y and middle_finger_tip.y < ring_finger_tip.y and \
                middle_finger_tip.y < pinky_tip.y:

                command = "girar_izquierda"

            elif ring_finger_tip.y < wrist.y and ring_finger_tip.y < thumb_tip.y and \
                ring_finger_tip.y < index_finger_tip.y and ring_finger_tip.y < middle_finger_tip.y and \
                ring_finger_tip.y < pinky_tip.y:
                
                command = "girar_derecha"

            elif pinky_tip.y < wrist.y and pinky_tip.y < thumb_tip.y and \
                pinky_tip.y < index_finger_tip.y and pinky_tip.y < middle_finger_tip.y and \
                pinky_tip.y < ring_finger_tip.y:
                
                command = "quieto"

            else:

                command = "quieto"

            # TODO Dibujar los landsmarks sobre la imagen
            mp_drawing.draw_landmarks(cv_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Mostrar en la esquina superior derecha el comando detectado
            cv2.putText(cv_image, command, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
            # Mostrar la imagen con los landmarks/gestos detectados
            cv2.imshow("Hand pose Estimation", cv_image)
            cv2.waitKey(1)

            # TODO Interpretar el gesto obtenido y enviar la orden de control ackermann
            ackermann_cmd = ackermann_msgs.msg.AckermannDrive()
            if command == "acelerar":
                ackermann_cmd.speed = 1.0  # Velocidad de avance
                ackermann_cmd.steering_angle = 0.0  # Mantener recto
            elif command == "frenar":
                ackermann_cmd.speed = -1.0  # Velocidad de retroceso
                ackermann_cmd.steering_angle = 0.0  # Mantener recto
            elif command == "quieto":
                ackermann_cmd.speed = 0.0  # Detener el coche
                ackermann_cmd.steering_angle = 0.0  # Mantener recto
            elif command == "girar_izquierda":
                ackermann_cmd.speed = 0.5  # Velocidad moderada
                ackermann_cmd.steering_angle = 1.0  # Girar a la izquierda
            elif command == "girar_derecha":
                ackermann_cmd.speed = 0.5  # Velocidad moderada
                ackermann_cmd.steering_angle = -1.0  # Girar a la derecha

            ackermann_command_publisher.publish(ackermann_cmd)# Publicar el mensaje de control
def main():
    global ackermann_command_publisher # Definir la variable global para publicar mensajes de control

    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)

    ## Publisher definition
    ackermann_command_publisher = rospy.Publisher(
            "/blue/preorder_ackermann_cmd",
            ackermann_msgs.msg.AckermannDrive,
            queue_size=10,
        )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
