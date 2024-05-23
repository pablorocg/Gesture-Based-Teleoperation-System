#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        # Inicializa el nodo de ROS
        rospy.init_node('obstacle_avoidance')

        # Suscriptor para la nube de puntos de los obstáculos captados por el lidar
        rospy.Subscriber("/obstacles", PointCloud2, self.obstacle_callback)

        # Suscriptor para comandos de control Ackermann
        rospy.Subscriber("/blue/preorder_ackermann_cmd", AckermannDrive, self.ackermann_callback)

        # TODO Publicador para comandos Ackermann modificados-----------------------------------------
        self.cmd_pub = rospy.Publisher("/blue/ackermann_cmd", AckermannDrive, queue_size=10)
        #---------------------------------------------------------------------------------------------
        # Almacenar el último mensaje Ackermann recibido
        self.last_ackermann_cmd = AckermannDrive()

    def obstacle_callback(self, msg):
        # TODO Procesar la nube de puntos con los obstáculos teniendo en cuenta el último mensaje de movimiento recibido para evitar colisiones-
        obstacles = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        collision = False
        for point in obstacles:
            x, y, z = point
            # Verificar si hay un obstáculo en la trayectoria
            if -1.6 < x < 1.6 and abs(y) < 1 and z > -0.8:
                collision = True
                break
        #----------------------------------------------------------------------------------------------------------------------------

        #Enviar mensaje de ackermann
        cmd = self.last_ackermann_cmd

        # TODO Modificar mensaje de ackermann si es necesario---------------------------------
        if collision:
            cmd = self.modify_ackermann_command()  
        #-------------------------------------------------------------------------------------

        self.cmd_pub.publish(cmd)
        
        return

    def ackermann_callback(self, msg):
        # Almacena el último comando recibido
        self.last_ackermann_cmd = msg

    def modify_ackermann_command(self):
        # Modifica el comando Ackermann para evitar obstáculos (Se puede modificar si es necesario)
        cmd = self.last_ackermann_cmd
        cmd.speed = -0.75  # Marcha atrás a poca velocidad
        cmd.steering_angle = 0.1 # Cambia ligeramente el ángulo de dirección
        return cmd
        

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()
