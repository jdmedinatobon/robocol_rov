#!/usr/bin/env python
import numpy as np
import rospy
import os

#Este script maestro es para crear los nodos sensores y como estan conectados.
#quizas volverlo un nodo? No se que haria.

#A es la matriz de adyacencia.
#Por ahora 3x3 y conecta el 1 con 2, el 2 con 3 y el 3 con 1.
A = np.array([[0, 1, 1],[1, 0, 1], [1, 1, 0]])

#TODO: Hacer esto, a partir de la matriz de adyacencia
vecinos_imu1 = "imu2 imu3"
vecinos_imu2 = "imu1 imu3"
vecinos_imu3 = "imu1 imu2"

#Por ahora chambon con rosrun
msg = "rosrun robocol_rov imu_node.py " + "imu1 " + vecinos_imu1
print(msg)
os.system("rosrun robocol_rov imu_node.py " + "imu1 " + vecinos_imu1)
#os.system("rosrun robocol_rov imu_node.py " + "imu2 " + vecinos_imu2)
#os.system("rosrun robocol_rov imu_node.py " + "imu3 " + vecinos_imu3)
