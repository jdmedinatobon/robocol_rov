#!/usr/bin/env python
import numpy as np
import rospy
import os

#Este script maestro es para crear los nodos sensores y como estan conectados.
#quizas volverlo un nodo? No se que haria.

#A es la matriz de adyacencia.
#Por ahora 3x3 y conecta el 1 con 2 y 3, el 2 con 3 y el 3 con 1.
A = np.array([[0, 1, 1],[1, 0, 0], [1, 0, 0]])

#TODO: Hacer esto con de la matriz de adyacencia
enlaces_imu1 = "link1 link2"
enlaces_imu2 = "link2"
enlaces_imu3 = "link1"

sensores_link1 = "imu1 imu3"
sensores_link2 = "imu1 imu2"

#Por ahora chambon con rosrun
os.system("rosrun robocol_rov imu_node.py " + "imu1 " + enlaces_imu1)
os.system("rosrun robocol_rov imu_node.py " + "imu2 " + enlaces_imu2)
os.system("rosrun robocol_rov imu_node.py " + "imu3 " + enlaces_imu3)

os.system("rosrun robocol_rov link_node.py " + "link1 " + sensores_link1)
os.system("rosrun robocol_rov link_node.py " + "link2 " + sensores_link2)
